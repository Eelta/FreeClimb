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
        else if(animationOnlyLeaf(i,lib.names,lib.parents))++oldVirtual;
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
    check(bool(actual)&&actual.count==91&&actual.virtualLeaves==6&&actual.unownedTracks==2,"installed NPC Root binds 91 body nodes, six virtual leaves, two engine-owned cameras");
    check(lookups[97]==0&&lookups[98]==0,"binding never looks up either engine-owned camera");
    std::vector<FixtureNode*> mapped;
    actual.each([&](std::size_t i,FixtureNode* node){
        check(i!=97&&i!=98&&scene.owned(node),"pose writes remain inside owned body and never touch cameras");
        check(node->name==lib.names[i],"actual hierarchy preserves animation track index");mapped.push_back(node);
    });
    check(mapped.size()==91,"only mapped body tracks are visited");
    const auto bridges=sceneBridgeNodes<FixtureNode*>(std::span<FixtureNode* const>(mapped.data(),mapped.size()),scene.root,[](FixtureNode* node){return node->parent;});
    check(bool(bridges)&&!bridges->empty(),"real XP32 parent chains require controller/adjustment bridges");
    bool adjustmentBridge=false;
    for(auto* node:*bridges) {
        check(scene.owned(node)&&node!=scene.root,"bridge collector stops at owned NPC Root");
        check(std::find(mapped.begin(),mapped.end(),node)==mapped.end(),"bridges are actual unmapped ancestors");
        check(node->name!=lib.names[97]&&node->name!=lib.names[98],"camera tracks cannot become body bridges");
        adjustmentBridge|=node->name.starts_with("CME ")||node->name.starts_with("MOV ");
    }
    check(adjustmentBridge,"fixture preserves real CME/MOV intermediate nodes");
    for(int i=0;i<99;++i) {
        if(animationOnlyLeaf(i,lib.names,lib.parents)||engineOwnedCameraTrack(i,lib.names,lib.parents))continue;
        auto missing=bindScene<FixtureNode*>(lib.names,lib.parents,[&](const std::string& name){return name==lib.names[i]?nullptr:scene.find(scene.root,name);});
        check(!missing&&missing.missing==i,"deleting each required body node from the actual lookup domain must fail");
    }
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
        check(bool(full)&&full.count==97&&full.virtualLeaves==0&&full.unownedTracks==2,"complete body binds while both camera tracks remain engine-owned");
        check(lookups[97]==0&&lookups[98]==0,"even present cameras are excluded before lookup");
        for(int i:{1,2,3})present.erase(lib.names[i]);
        auto partial=bindScene<int*>(lib.names,lib.parents,lookup);
        check(bool(partial)&&partial.count==94&&partial.virtualLeaves==3&&partial.unownedTracks==2,"standard scene without three animation-only leaves binds");
        std::size_t visits=0;partial.each([&](std::size_t i,int* node){check(node==&objects[i],"preserve track indices");*node=int(i)+100;++visits;});
        check(visits==94&&objects[1]==0&&objects[2]==0&&objects[3]==0&&objects[97]==0&&objects[98]==0,"no read/write visits an absent helper or engine-owned camera");
        check(objects[4]==104&&objects[38]==138&&objects[96]==196,"COM, hand and fingers must not shift by three tracks");
        for(int i:{42,43,60})present.erase(lib.names[i]);
        partial=bindScene<int*>(lib.names,lib.parents,lookup);
        check(bool(partial)&&partial.count==91&&partial.virtualLeaves==6&&partial.unownedTracks==2,"missing unequipped attachment nodes do not reject the body");
        present.erase(lib.names[97]);present.erase(lib.names[98]);
        check(bool(bindScene<int*>(lib.names,lib.parents,lookup)),"absence of engine camera nodes cannot disable body binding");
        for(int i=0;i<99;++i) {
            if(animationOnlyLeaf(i,lib.names,lib.parents)||engineOwnedCameraTrack(i,lib.names,lib.parents))continue;
            present.erase(lib.names[i]);auto invalid=bindScene<int*>(lib.names,lib.parents,lookup);
            check(!invalid&&invalid.missing==i,"every non-helper scene bone remains mandatory");present.insert(lib.names[i]);
        }
        auto changed=lib.parents;changed[4]=1;
        check(!bindScene<int*>(lib.names,changed,lookup),"a helper cannot be optional when an anatomical bone inherits from it");
        auto names=lib.names;names[1]="x_unknown";
        check(!bindScene<int*>(names,lib.parents,lookup),"arbitrary x_ names do not bypass validation");
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
        std::cout<<"PASS: owned hierarchy, exact camera schema, required body bones, stable indices, null-safe iteration\n";
    } catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}
}
