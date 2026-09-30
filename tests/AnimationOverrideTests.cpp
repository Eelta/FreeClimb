#include "AnimationOverrides.h"
#include "HkxFixtures.h"
#include <chrono>
#include <cstring>
#include <iostream>
#include <limits>

using namespace fc;
namespace {
unsigned checks=0;
void check(bool value,const std::string& why){++checks;if(!value)throw std::runtime_error(why);}
void equalVec(Vec a,Vec b,const std::string& why){check(a.x==b.x&&a.y==b.y&&a.z==b.z,why);}
std::vector<std::uint8_t> bytes(const std::filesystem::path& p){std::ifstream f(p,std::ios::binary);return {std::istreambuf_iterator<char>(f),{}};}
struct Temp {
    std::filesystem::path path;
    Temp(){path=std::filesystem::current_path()/("hkx-override-test-"+std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));check(std::filesystem::create_directory(path),"fresh test directory");}
    ~Temp(){std::error_code ec;std::filesystem::remove_all(path,ec);}
};
void save(const std::filesystem::path& dir,Motion m,const hkx_fixture::Spec& s){hkx_fixture::save(dir/(std::string(motionSlotNames[int(m)-1])+".hkx"),hkx_fixture::pack(s).bytes);}
void counts(const AnimationOverrideReport& r,unsigned loaded,unsigned rejected,unsigned missing){
    check(r.slots.size()==activeMotionCount&&r.loaded==loaded&&r.rejected==rejected&&r.missing==missing,"slot counters differ: loaded="+std::to_string(r.loaded)+" rejected="+std::to_string(r.rejected)+" missing="+std::to_string(r.missing));
    for(std::size_t i=0;i<activeMotions.size();++i)check(r.slots[i].motion==activeMotions[i],"report keeps original active numeric IDs");
}
void calibration(const Library& a,const Library& b){
    Settings x,y;check(a.configureThreepeat(x)&&b.configureThreepeat(y),"base calibration available");
    check(x.threepeatHangHeight==y.threepeatHangHeight&&x.threepeatHandHalfWidth==y.threepeatHandHalfWidth&&x.threepeatHangForward==y.threepeatHangForward,"override changed hang calibration");
    check(x.threepeatHopDistance==y.threepeatHopDistance&&x.threepeatHopSeconds==y.threepeatHopSeconds,"override changed hop calibration");
    check(x.threepeatMantleHeight==y.threepeatMantleHeight&&x.threepeatMantleSeconds==y.threepeatMantleSeconds&&x.threepeatMantleForward==y.threepeatMantleForward,"override changed mantle calibration");
    for(int i=0;i<2;++i){equalVec(x.threepeatHangToes[i],y.threepeatHangToes[i],"toe calibration");equalVec(x.threepeatMantleReplant[i],y.threepeatMantleReplant[i],"replant calibration");}
}
void protectedData(const Library& base,const Library& value){
    check(base.names==value.names&&base.parents==value.parents,"skeleton changed");
    check(base.sourceFingerprint==value.sourceFingerprint&&base.sourceBytes==value.sourceBytes&&value.sourceValidated,"base fingerprint changed");
    for(int i=0;i<motionCount;++i){
        const auto& a=base.clips[i];const auto& b=value.clips[i];
        check(a.seconds==b.seconds&&a.stride==b.stride&&a.height==b.height&&a.contacts==b.contacts,"base clip metadata/contact changed");equalVec(a.travel,b.travel,"root travel changed");
        check(a.frames.size()==b.frames.size(),"base frame count changed");
        for(unsigned f=0;f<a.frames.size();++f)for(unsigned bone=0;bone<99;++bone){
            equalVec(a.frames[f][bone].t,b.frames[f][bone].t,"authored translation changed");equalVec(a.frames[f][bone].s,b.frames[f][bone].s,"authored scale changed");
            check(angleBetween(a.frames[f][bone].q,b.frames[f][bone].q)<1e-6f,"raw base quaternion changed");
        }
    }
    calibration(base,value);
}
void legacyLayouts(const std::vector<std::uint8_t>& original) {
    auto integer=[&](std::size_t offset) {
        check(offset+4<=original.size(),"legacy fixture field in bounds");
        std::uint32_t result{};std::memcpy(&result,original.data()+offset,4);return result;
    };
    check(integer(8)==99&&integer(12)==42,"legacy fixture has original 42 slots");
    std::size_t offset=16;
    for(int bone=0;bone<99;++bone)offset+=8+integer(offset+4)+sizeof(Transform);
    std::size_t retiredPayload=0,retiredContacts=0;
    for(int clip=0;clip<legacyMotionCount;++clip) {
        const auto frames=integer(offset+4);offset+=28;
        if(clip==11){retiredPayload=offset;retiredContacts=offset+99*sizeof(Transform);}
        offset+=std::size_t(frames)*(99*sizeof(Transform)+4*sizeof(float));
        check(offset<=original.size(),"legacy fixture clip in bounds");
    }
    Temp temp;auto historical=original;historical.resize(offset);hkx_fixture::put(historical,12,std::uint32_t(legacyMotionCount));
    const auto file=temp.path/"historical.motion";hkx_fixture::save(file,historical);
    Library library;check(library.loadLegacy(file.string())&&!library.hasThreepeat(),"Historical 38-record binary remains available for offline migration");
    for(int id=1;id<=motionCount;++id)
        check(library.clips[id-1].frames.empty()==(!isActiveMotion(Motion(id))||id>legacyMotionCount),"Historical binary stores only active available slots");
    auto bad=original;hkx_fixture::put(bad,retiredPayload,std::numeric_limits<float>::quiet_NaN());hkx_fixture::save(file,bad);
    check(!library.loadLegacy(file.string()),"Discarded retired transforms are still validated");
    bad=original;hkx_fixture::put(bad,retiredContacts,2.f);hkx_fixture::save(file,bad);
    check(!library.loadLegacy(file.string()),"Discarded retired contact weights are still validated");
}
}
int main(int argc,char** argv) try {
    check(argc==3,"arguments: baseline motion, exported HKX directory");const auto motionPath=std::filesystem::path(argv[1]);const auto before=bytes(motionPath);
    Library base;check(base.load(motionPath.string()),"load actual baseline");check(base.sourceBytes==9975522&&base.sourceFingerprint==0x3ee1dcacbb182677ull,"expected frozen 42-motion fingerprint");
    legacyLayouts(before);
    for(int id:{6,7,12,13,14,33,38}) {
        check(base.clips[id-1].frames.empty()&&base.clips[id-1].contacts.empty(),"Retired legacy payload must not allocate retained animation data");
        const auto sampled=base.sample(Motion(id),.5f);
        check(sampled.size()==base.rest.size()&&base.clip(Motion(id)).frames.empty(),"Retired legacy lookup returns no clip instead of clamping");
        for(std::size_t bone=0;bone<sampled.size();++bone) {
            equalVec(sampled[bone].t,base.rest[bone].t,"Retired lookup returns rest translation");
            check(angleBetween(sampled[bone].q,base.rest[bone].q)<1e-6f,"Retired lookup returns rest rotation");
        }
    }
    Temp empty;Library value=base;counts(loadHkxOverrides(value,empty.path),0,0,activeMotionCount);
    for(int i=1;i<=motionCount;++i)check(!value.hasAnimationOverride(Motion(i)),"missing slot installed override");
    auto full=loadHkxOverrides(value,argv[2]);counts(full,activeMotionCount,0,0);protectedData(base,value);
    float worst=0;int worstSlot=0,worstBone=0;float worstPhase=0;
    for(const auto motion:activeMotions)for(int frame=0;frame<=120;++frame){
        const int i=int(motion);
        float phase=frame/120.f;auto a=base.sample(Motion(i),phase),b=value.sample(Motion(i),phase);
        check(base.contactWeights(Motion(i),phase)==value.contactWeights(Motion(i),phase),"sample contact changed");
        for(int bone=0;bone<99;++bone){
            equalVec(a[bone].t,b[bone].t,"sample translation changed");equalVec(a[bone].s,b[bone].s,"sample scale changed");
            float angle=angleBetween(a[bone].q,b[bone].q);if(angle>worst){worst=angle;worstSlot=i;worstBone=bone;worstPhase=phase;}
            check(angle<.00174533f,"exported rotation exceeds .1 degree equivalence bound: slot "+std::to_string(i)+" bone "+std::to_string(bone));
            if(bone<5||bone>=97)check(angle<1e-6f,"protected root/control/camera rotation changed");
        }
    }
    std::cout<<activeMotionCount<<" comparable-slot sample equivalence max radians="<<worst<<" slot="<<worstSlot<<" bone="<<worstBone<<" phase="<<worstPhase<<'\n';
    counts(loadHkxOverrides(value,empty.path),0,0,activeMotionCount);for(int i=1;i<=motionCount;++i)check(!value.hasAnimationOverride(Motion(i)),"reload retained stale slot");
    for(const char* name:{"mantle","step","toFree","toBraced","freeHang","runDown","dropCatch","contextRegrab",""})hkx_fixture::save(empty.path/(std::string(name)+".hkx"),std::vector<std::uint8_t>{1,2,3});
    counts(loadHkxOverrides(value,empty.path),0,0,activeMotionCount);
    for(int id:{6,7,12,13,14,33,38})check(value.rotationOverrides[id-1].frames.empty(),"Retired and blank slot filenames are never loaded or reported");
    Temp sparse;auto spec=hkx_fixture::procedural({96,38,0,4,97,98,69},5);for(int n:spec.indices)spec.names.push_back(base.names[n]);
    save(sparse.path,Motion::contextHang,spec);value=base;counts(loadHkxOverrides(value,sparse.path),1,0,activeMotionCount-1);calibration(base,value);
    for(float phase:{0.f,.23f,.5f,.88f,1.f}){
        auto a=base.sample(Motion::contextHang,phase),b=value.sample(Motion::contextHang,phase);
        for(int bone=0;bone<99;++bone){
            equalVec(a[bone].t,b[bone].t,"sparse root/translation");equalVec(a[bone].s,b[bone].s,"sparse scale");
            auto found=std::find(spec.indices.begin(),spec.indices.end(),bone);
            if(bone>=5&&bone<97&&found!=spec.indices.end()){
                auto track=std::size_t(found-spec.indices.begin());float position=phase*4;auto first=std::min(std::size_t(position),std::size_t(3));
                auto expected=blend(spec.frames[first][track].q,spec.frames[first+1][track].q,position-float(first));
                check(angleBetween(b[bone].q,expected)<1e-5f,"sparse mapping or double finger basis correction");
            } else check(angleBetween(a[bone].q,b[bone].q)<1e-6f,"sparse override leaked to untargeted bone");
        }
    }
    auto broken=hkx_fixture::pack(spec);broken.bytes[0]=0;hkx_fixture::save(sparse.path/"contextHang.hkx",broken.bytes);
    save(sparse.path,Motion::up,hkx_fixture::procedural({28},3));counts(loadHkxOverrides(value,sparse.path),1,1,activeMotionCount-2);
    check(value.hasAnimationOverride(Motion::up)&&!value.hasAnimationOverride(Motion::contextHang),"illegal slot did not fall back independently");
    Temp names;auto conflict=hkx_fixture::procedural({28});conflict.names={"not-canonical"};save(names.path,Motion::hang,conflict);counts(loadHkxOverrides(value,names.path),0,1,activeMotionCount-1);
    conflict=hkx_fixture::procedural({0,1,4,97,98});save(names.path,Motion::hang,conflict);counts(loadHkxOverrides(value,names.path),0,1,activeMotionCount-1);
    std::vector<int> canonical126(126);std::iota(canonical126.begin(),canonical126.end(),0);auto extended=hkx_fixture::procedural(canonical126,2);
    extended.identityMap=true;for(int i=0;i<126;++i)extended.names.push_back(i<99?base.names[i]:"original-extra-"+std::to_string(i));
    save(names.path,Motion::hang,extended);counts(loadHkxOverrides(value,names.path),1,0,activeMotionCount-1);
    check(value.sample(Motion::hang,.5f).size()==99,"extended skeleton changed output size");
    extended.names.clear();save(names.path,Motion::hang,extended);counts(loadHkxOverrides(value,names.path),0,1,activeMotionCount-1);
    auto extra=hkx_fixture::procedural({255,28},2);save(names.path,Motion::hang,extra);counts(loadHkxOverrides(value,names.path),1,0,activeMotionCount-1);
    Temp limitsDir;auto normal=hkx_fixture::procedural({28},3);save(limitsDir.path,Motion::hang,normal);save(limitsDir.path,Motion::up,normal);
    AnimationOverrideLimits limits;limits.fileBytes=207;counts(loadHkxOverrides(value,limitsDir.path,limits),0,2,activeMotionCount-2);
    limits={};limits.totalBytes=std::filesystem::file_size(limitsDir.path/"hang.hkx");counts(loadHkxOverrides(value,limitsDir.path,limits),1,1,activeMotionCount-2);
    limits={};limits.totalOutputBytes=base.clips[0].frames.size()*sizeof(std::array<Quat,99>);counts(loadHkxOverrides(value,limitsDir.path,limits),1,1,activeMotionCount-2);
    limits={};limits.totalOutputBytes=0;counts(loadHkxOverrides(value,limitsDir.path,limits),0,2,activeMotionCount-2);
    limits={};limits.frames=2;counts(loadHkxOverrides(value,limitsDir.path,limits),0,2,activeMotionCount-2);
    limits={};limits.fileMilliseconds=0;counts(loadHkxOverrides(value,limitsDir.path,limits),0,2,activeMotionCount-2);
    limits={};limits.totalMilliseconds=0;counts(loadHkxOverrides(value,limitsDir.path,limits),0,2,activeMotionCount-2);
    Library unavailable;counts(loadHkxOverrides(unavailable,limitsDir.path),0,2,activeMotionCount-2);
    Temp nonfile;std::filesystem::create_directory(nonfile.path/"hang.hkx");counts(loadHkxOverrides(value,nonfile.path),0,1,activeMotionCount-1);
    check(bytes(motionPath)==before,"baseline.motion bytes changed");
    std::cout<<"HKX override policy: "<<checks<<" checks passed\n";return 0;
} catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}
