#include "Pose.h"
#include "SceneBinding.h"
#include "ScenePropagation.h"
#include <fstream>
#include <iostream>
#include <map>
#include <set>
#include <sstream>
#include <stdexcept>
using namespace fc;
constexpr std::array<int,9> equipmentTracks{42,43,60,61,62,63,64,65,66};
bool unownedTrack(int i){return i>=97||std::find(equipmentTracks.begin(),equipmentTracks.end(),i)!=equipmentTracks.end();}
void check(bool ok,const char* message){if(!ok)throw std::runtime_error(message);}
struct FixtureNode {
    int id{},parentId{-1},order{};
    std::string name;
    FixtureNode* parent{};
    std::vector<FixtureNode*> children;
};
struct FixtureScene {
    std::map<int,FixtureNode> nodes;
    FixtureNode* root{};
    explicit FixtureScene(const char* path) {
        std::ifstream input(path);check(bool(input),"actual NIF hierarchy must exist");
        auto line=[&](){std::string value;std::getline(input,value);if(!value.empty()&&value.back()=='\r')value.pop_back();return value;};
        check(line()=="FREECLIMB_SCENE_HIERARCHY_V1","hierarchy format must preserve actual parent references");
        const auto header=line();check(header.starts_with("root\t"),"fixture must identify owned NPC Root");
        const int rootId=std::stoi(header.substr(5));
        while(input.peek()!=std::char_traits<char>::eof()) {
            const auto record=line();if(record.empty())continue;
            std::istringstream fields(record);std::string kind,id,parent,order,name;
            std::getline(fields,kind,'\t');std::getline(fields,id,'\t');std::getline(fields,parent,'\t');
            std::getline(fields,order,'\t');std::getline(fields,name);
            check(kind=="node"&&!id.empty()&&!parent.empty()&&!order.empty(),"hierarchy record fields");
            FixtureNode node;node.id=std::stoi(id);node.parentId=std::stoi(parent);node.order=std::stoi(order);node.name=name;
            check(nodes.emplace(node.id,std::move(node)).second,"NIF block IDs must be unique");
        }
        check(nodes.contains(rootId),"declared NPC Root block must exist");root=&nodes.at(rootId);
        check(root->name=="NPC Root [Root]","owned root identity");
        for(auto& [id,node]:nodes)if(node.parentId>=0) {
            check(nodes.contains(node.parentId),"all retained parent references must resolve");
            node.parent=&nodes.at(node.parentId);node.parent->children.push_back(&node);
        }
        for(auto& [id,node]:nodes)std::sort(node.children.begin(),node.children.end(),[](auto* a,auto* b){return a->order<b->order;});
        for(auto& [id,node]:nodes) {
            std::set<FixtureNode*> ancestors;auto* current=&node;
            while(current){check(ancestors.insert(current).second,"NIF parent chain cannot cycle");current=current->parent;}
        }
    }
    FixtureNode* find(FixtureNode* start,const std::string& name,unsigned depth=0) const {
        if(!start||depth>128)return nullptr;
        if(start->name==name)return start;
        for(auto* child:start->children)if(auto* found=find(child,name,depth+1))return found;
        return nullptr;
    }
    bool owned(FixtureNode* node) const {
        for(unsigned depth=0;node&&depth<256;++depth,node=node->parent)if(node==root)return true;
        return false;
    }
};
void actualScene(const char* path,const Library& lib) {
    FixtureScene scene(path);
    bool externalCamera=false;
    for(auto& [id,node]:scene.nodes)if(node.name==lib.names[97]&&!scene.owned(&node))externalCamera=true;
    check(externalCamera&&!scene.find(scene.root,lib.names[97]),"real Camera3rd exists only outside owned NPC Root");
    check(scene.find(scene.root,lib.names[98]),"real Camera Control is present but remains engine-owned");

    int oldMissing=-1;std::size_t oldMapped=0,oldVirtual=0;
    for(int i=0;i<99;++i) {
        if(scene.find(scene.root,lib.names[i]))++oldMapped;
        else if(i==1||i==2||i==3||i==42||i==43||i==60)++oldVirtual;
        else {oldMissing=i;break;}
    }
    check(oldMissing==97&&oldMapped==91&&oldVirtual==6,"old six-optional policy reproduces actual index97 mapped91 failure");
    std::array<unsigned,99> lookups{};
    auto lookup=[&](const std::string& name)->FixtureNode* {
        const auto it=std::find(lib.names.begin(),lib.names.end(),name);
        check(it!=lib.names.end(),"known animation lookup name");++lookups[it-lib.names.begin()];
        return scene.find(scene.root,name);
    };
    const auto actual=bindScene<FixtureNode*>(lib.names,lib.parents,lookup);
    check(bool(actual)&&actual.count==85&&actual.virtualLeaves==3&&actual.unownedTracks==11,"installed NPC Root binds 85 body nodes, three virtual helpers, eleven engine-owned tracks");
    for(int i=0;i<99;++i)if(unownedTrack(i))check(lookups[i]==0&&!actual.nodes[i],"binding never looks up or stores engine-owned equipment or cameras");
    std::vector<FixtureNode*> mapped;
    actual.each([&](std::size_t i,FixtureNode* node){
        check(!unownedTrack(int(i))&&scene.owned(node),"pose writes remain inside owned body and never touch equipment or cameras");
        check(node->name==lib.names[i],"actual hierarchy preserves animation track index");mapped.push_back(node);
    });
    check(mapped.size()==85,"only mapped body tracks are visited");
    const auto bridges=sceneBridgeNodes<FixtureNode*>(std::span<FixtureNode* const>(mapped.data(),mapped.size()),scene.root,[](FixtureNode* node){return node->parent;});
    check(bool(bridges)&&!bridges->empty(),"real XP32 parent chains require controller/adjustment bridges");
    bool adjustmentBridge=false;
    for(auto* node:*bridges) {
        check(scene.owned(node)&&node!=scene.root,"bridge collector stops at owned NPC Root");
        check(std::find(mapped.begin(),mapped.end(),node)==mapped.end(),"bridges are actual unmapped ancestors");
        for(int i=0;i<99;++i)if(unownedTrack(i))check(node->name!=lib.names[i],"unrelated equipment and camera tracks cannot become body bridges");
        adjustmentBridge|=node->name.starts_with("CME ")||node->name.starts_with("MOV ");
    }
    check(adjustmentBridge,"fixture preserves real CME/MOV intermediate nodes");
    for(int i=0;i<99;++i) {
        if(animationOnlyLeaf(i,lib.names,lib.parents)||unownedTrack(i))continue;
        auto missing=bindScene<FixtureNode*>(lib.names,lib.parents,[&](const std::string& name){return name==lib.names[i]?nullptr:scene.find(scene.root,name);});
        check(!missing&&missing.missing==i,"deleting each required body node from the actual lookup domain must fail");
    }
    auto* axe=scene.find(scene.root,lib.names[61]);auto* spine=scene.find(scene.root,lib.names[26]);
    check(axe&&spine&&axe->parent&&axe->children.empty(),"real fixture provides a movable WeaponAxe leaf and mapped spine");
    auto& siblings=axe->parent->children;siblings.erase(std::find(siblings.begin(),siblings.end(),axe));
    axe->parent=spine;axe->parentId=spine->id;spine->children.push_back(axe);
    check(spine!=scene.find(scene.root,lib.names[lib.parents[61]])&&actual.nodes[26]==spine,"reparented WeaponAxe encounters another mapped bone before its canonical pelvis parent");
    lookups.fill(0);const auto reparented=bindScene<FixtureNode*>(lib.names,lib.parents,lookup);
    check(bool(reparented)&&reparented.nodes==actual.nodes&&lookups[61]==0,"reported equipment reparenting preserves the exact anatomical binding without looking up WeaponAxe");
    std::cout<<"PASS actual hierarchy: "<<path<<" root="<<scene.root->id<<" mapped="<<actual.count
        <<" virtual="<<actual.virtualLeaves<<" engineOwned="<<actual.unownedTracks<<" bridges="<<bridges->size()<<" legacyMissing="<<oldMissing<<'\n';
}
int main(int argc,char** argv) {
    try {
        check(argc>=2,"motion path is required");Library lib;check(lib.load(argv[1]),"motion library");
        std::array<int,99> objects{};std::array<unsigned,99> lookups{};
        std::set<std::string> present(lib.names.begin(),lib.names.end());
        auto lookup=[&](const std::string& name)->int* {
            const auto it=std::find(lib.names.begin(),lib.names.end(),name);
            if(it==lib.names.end())return nullptr;
            const auto i=it-lib.names.begin();++lookups[i];
            return present.contains(name)?&objects[i]:nullptr;
        };
        auto full=bindScene<int*>(lib.names,lib.parents,lookup);
        check(bool(full)&&full.count==88&&full.virtualLeaves==0&&full.unownedTracks==11,"complete body binds while equipment and camera tracks remain engine-owned");
        for(int i=0;i<99;++i)if(unownedTrack(i))check(lookups[i]==0&&!full.nodes[i],"present equipment and cameras are excluded before lookup");
        for(int i:{1,2,3})present.erase(lib.names[i]);
        auto partial=bindScene<int*>(lib.names,lib.parents,lookup);
        check(bool(partial)&&partial.count==85&&partial.virtualLeaves==3&&partial.unownedTracks==11,"standard scene without three animation-only leaves binds");
        std::size_t visits=0;partial.each([&](std::size_t i,int* node){check(node==&objects[i],"preserve track indices");*node=int(i)+100;++visits;});
        check(visits==85&&objects[1]==0&&objects[2]==0&&objects[3]==0,"no read/write visits an absent helper");
        for(int i=0;i<99;++i)if(unownedTrack(i))check(objects[i]==0&&lookups[i]==0,"no read/write or lookup visits engine-owned equipment or cameras");
        check(objects[4]==104&&objects[38]==138&&objects[96]==196,"COM, hand and fingers must not shift by three tracks");
        for(int i:equipmentTracks)present.erase(lib.names[i]);
        partial=bindScene<int*>(lib.names,lib.parents,lookup);
        check(bool(partial)&&partial.count==85&&partial.virtualLeaves==3&&partial.unownedTracks==11,"all absent equipment nodes retain the same anatomical binding");
        present.erase(lib.names[97]);present.erase(lib.names[98]);
        check(bool(bindScene<int*>(lib.names,lib.parents,lookup)),"absence of engine camera nodes cannot disable body binding");
        for(int i=0;i<99;++i) {
            if(animationOnlyLeaf(i,lib.names,lib.parents)||unownedTrack(i))continue;
            present.erase(lib.names[i]);auto invalid=bindScene<int*>(lib.names,lib.parents,lookup);
            check(!invalid&&invalid.missing==i,"every non-helper scene bone remains mandatory");present.insert(lib.names[i]);
        }
        auto changed=lib.parents;changed[4]=1;
        check(!bindScene<int*>(lib.names,changed,lookup),"a helper cannot be optional when an anatomical bone inherits from it");
        auto names=lib.names;names[1]="x_unknown";
        check(!bindScene<int*>(names,lib.parents,lookup),"arbitrary x_ names do not bypass validation");
        for(int equipment:equipmentTracks) {
            check(engineOwnedEquipmentTrack(equipment,lib.names,lib.parents),"all nine canonical equipment leaves remain native");
            check(engineOwnedEquipmentName(lib.names[equipment],lib.names,lib.parents),"equipment ancestor guard recognizes each native equipment name");
            check(!engineOwnedEquipmentName("x_"+lib.names[equipment],lib.names,lib.parents),"equipment ancestor guard never uses fuzzy names");
            names=lib.names;changed=lib.parents;std::swap(names[1],names[equipment]);std::swap(changed[1],changed[equipment]);
            check(!engineOwnedEquipmentTrack(1,names,changed)&&!engineOwnedEquipmentName(lib.names[equipment],names,changed),"equipment identity requires its exact canonical index");
            names=lib.names;names[equipment]="Unknown equipment";
            check(!engineOwnedEquipmentTrack(equipment,names,lib.parents),"equipment exclusions require exact canonical names");
            auto invalid=bindScene<int*>(names,lib.parents,lookup);
            check(!invalid&&invalid.missing==equipment,"unknown equipment identity remains required");
            for(int parent:{-1,0,4,99})if(parent!=lib.parents[equipment]) {
                changed=lib.parents;changed[equipment]=parent;
                check(!engineOwnedEquipmentTrack(equipment,lib.names,changed)&&!engineOwnedEquipmentName(lib.names[equipment],lib.names,changed),"equipment exclusions require exact canonical parents");
                invalid=bindScene<int*>(lib.names,changed,lookup);
                check(!invalid&&invalid.missing==equipment,"changed canonical equipment parent cannot be silently excluded");
            }
            changed=lib.parents;changed[96]=equipment;
            check(!engineOwnedEquipmentTrack(equipment,lib.names,changed)&&!engineOwnedEquipmentName(lib.names[equipment],lib.names,changed),"equipment that becomes a body ancestor cannot be excluded");
            invalid=bindScene<int*>(lib.names,changed,lookup);
            check(!invalid&&invalid.missing==equipment,"a required anatomical ancestor cannot be omitted");
        }
        for(int i=0;i<99;++i)if(std::find(equipmentTracks.begin(),equipmentTracks.end(),i)==equipmentTracks.end())
            check(!engineOwnedEquipmentTrack(i,lib.names,lib.parents)&&!engineOwnedEquipmentName(lib.names[i],lib.names,lib.parents),"only the nine recognized equipment tracks are excluded");
        check(!engineOwnedEquipmentTrack(99,lib.names,lib.parents),"out-of-range equipment index rejected");
        auto shortNames=lib.names;shortNames.pop_back();auto shortParents=lib.parents;shortParents.pop_back();
        check(!engineOwnedEquipmentName("WeaponAxe",shortNames,lib.parents)&&!engineOwnedEquipmentName("WeaponAxe",lib.names,shortParents),"equipment ancestor guard rejects incomplete canonical layouts");
        check(!engineOwnedEquipmentName("",lib.names,lib.parents)&&!engineOwnedEquipmentName("WeaponAxeExtra",lib.names,lib.parents),"equipment ancestor guard rejects empty and partial names");
        for(int camera:{97,98}) {
            names=lib.names;names[camera]="Unknown camera";
            check(!engineOwnedCameraTrack(camera,names,lib.parents),"unknown camera identity cannot be excluded");
            auto invalid=bindScene<int*>(names,lib.parents,lookup);
            check(!invalid&&invalid.missing==camera,"unknown track at a camera index remains required");
            for(int parent:{-1,0,4,99})if(parent!=lib.parents[camera]) {
                changed=lib.parents;changed[camera]=parent;
                check(!engineOwnedCameraTrack(camera,lib.names,changed),"changed camera parent cannot be excluded");
                invalid=bindScene<int*>(lib.names,changed,lookup);
                check(!invalid&&invalid.missing==camera,"camera parent schema mismatch stays required");
            }
            changed=lib.parents;changed[4]=camera;
            check(!engineOwnedCameraTrack(camera,lib.names,changed),"camera cannot be excluded if a body bone inherits from it");
            invalid=bindScene<int*>(lib.names,changed,lookup);
            check(!invalid&&invalid.missing==camera,"a missing ancestor of the body is never optional");
        }
        for(int file=2;file<argc;++file)actualScene(argv[file],lib);
        std::cout<<"PASS: owned hierarchy, exact equipment and camera schema, required body bones, stable indices, null-safe iteration\n";
    } catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}
}
