#include "RayCollectorValidationCache.h"
#include <cstddef>
#include <cstdlib>
#include <iostream>
#include <memory>

namespace {
unsigned checks{};
void require(bool value,const char* message) {
    ++checks;
    if(!value){std::cerr<<"FAIL "<<message<<'\n';std::exit(1);}
}
void nativeAdd(){}
using Add=void(*)();
using Validation=fc::RayCollectorValidation<Add>;
struct Table {std::uintptr_t pick=7;bool readable=true;};
struct World:std::enable_shared_from_this<World> {
    Table* table{};
    bool readable=true;
    unsigned* destroyed{};
    explicit World(Table* value,unsigned* counter=nullptr):table(value),destroyed(counter){}
    ~World(){if(destroyed)++*destroyed;}
};
struct WorldHandle {
    using element_type=World;
    std::shared_ptr<World> value;
    WorldHandle()=default;
    explicit WorldHandle(World* world):value(world?world->shared_from_this():nullptr){}
    World* get()const{return value.get();}
    explicit operator bool()const{return bool(value);}
    void reset(){value.reset();}
};
using Cache=fc::RayCollectorValidationCache<WorldHandle,Add>;
struct Probe {
    unsigned verifies{},pickReads{};
    Add resolve(Cache& cache,World* world) {
        cache.bind(world);
        const auto table=world?reinterpret_cast<std::uintptr_t>(world->table):0;
        return cache.resolve(table,[&](std::uintptr_t address) {
            ++pickReads;
            const auto* candidate=reinterpret_cast<const Table*>(address);
            require(candidate&&candidate->readable,"only a validated readable table can have its Pick slot inspected");
            return candidate->pick;
        },[&](World* candidate,std::uintptr_t address)->Validation {
            ++verifies;
            if(!candidate->readable||reinterpret_cast<std::uintptr_t>(candidate->table)!=address)return {};
            const auto* candidateTable=reinterpret_cast<const Table*>(address);
            return candidateTable&&candidateTable->readable&&candidateTable->pick==7?Validation{nativeAdd,7}:Validation{};
        });
    }
};
void oneValidationPerWorld() {
    Table table;
    auto world=std::make_shared<World>(&table);
    Cache cache;Probe probe;
    for(unsigned ray=0;ray<4170;++ray)require(probe.resolve(cache,world.get())==nativeAdd,"same world retains a valid collector");
    require(probe.verifies==1,"4170 rays perform one full world validation");
    require(probe.pickReads==4169,"every successful cache hit still inspects the Pick slot");
}
void failedWorldAndTableAreCached() {
    Table good,bad;bad.readable=false;
    auto first=std::make_shared<World>(&good),second=std::make_shared<World>(&good);
    Cache cache;Probe probe;
    require(probe.resolve(cache,first.get())==nativeAdd,"first world is valid");
    second->readable=false;
    require(!probe.resolve(cache,second.get()),"a new unreadable world does not inherit successful validation");
    const auto reads=probe.pickReads;
    for(unsigned i=0;i<20;++i)require(!probe.resolve(cache,second.get()),"invalid world remains blocking");
    require(probe.verifies==2&&probe.pickReads==reads,"a failed world is cached without inspecting its table");
    second->table=&bad;
    require(!probe.resolve(cache,second.get()),"a changed vtable triggers another failed validation");
    require(probe.verifies==3,"changed vtable invalidates a cached failure");
    for(unsigned i=0;i<20;++i)require(!probe.resolve(cache,second.get()),"invalid vtable remains blocking");
    require(probe.verifies==3&&probe.pickReads==reads,"failed vtable is not inspected or verified repeatedly");
    second->readable=true;second->table=&good;
    require(probe.resolve(cache,second.get())==nativeAdd,"a new valid table can restore filtering");
    second->table=nullptr;
    require(!probe.resolve(cache,second.get()),"null table cannot reuse success");
    require(!probe.resolve(cache,second.get()),"null table failure is cached");
    require(probe.verifies==5,"null table is verified only once");
}
void changedTableAndPickInvalidateSuccess() {
    Table first,second;second.readable=false;
    auto world=std::make_shared<World>(&first);
    Cache cache;Probe probe;
    require(probe.resolve(cache,world.get())==nativeAdd,"original vtable is valid");
    world->table=&second;
    require(!probe.resolve(cache,world.get()),"new unreadable vtable cannot reuse old success");
    require(probe.verifies==2,"new vtable is fully validated");
    world->table=&first;
    require(probe.resolve(cache,world.get())==nativeAdd,"returning to the original table revalidates");
    first.pick=99;
    require(!probe.resolve(cache,world.get()),"Pick hotpatch at the same vtable address invalidates success");
    require(probe.verifies==4,"changed Pick slot gets full validation");
    const auto reads=probe.pickReads;
    require(!probe.resolve(cache,world.get()),"Pick validation failure is cached");
    first.pick=7;
    require(!probe.resolve(cache,world.get()),"failed table stays blocking until next instance or identity change");
    require(probe.verifies==4&&probe.pickReads==reads,"failed cache never probes unvalidated Pick memory");
    Cache nextFrame;
    require(probe.resolve(nextFrame,world.get())==nativeAdd,"next GameWorld retries the restored Pick slot");
    require(probe.verifies==5,"a new GameWorld never inherits a cached failure");
}
void cacheStoresOnlyTheValidatedPick() {
    Table table;
    auto world=std::make_shared<World>(&table);
    Cache cache;cache.bind(world.get());
    unsigned verifies{};
    const auto readPick=[&](std::uintptr_t){return table.pick;};
    const auto verify=[&](World*,std::uintptr_t)->Validation {
        ++verifies;
        if(table.pick!=7)return {};
        const Validation result{nativeAdd,table.pick};
        table.pick=99;
        return result;
    };
    const auto address=reinterpret_cast<std::uintptr_t>(&table);
    require(cache.resolve(address,readPick,verify)==nativeAdd,"validation returns the collector it just verified");
    require(!cache.resolve(address,readPick,verify),"a patch after validation is detected on the next ray");
    require(verifies==2,"post-validation Pick must not become the cached successful identity");
}
void newFrameAndMissingWorld() {
    Table table;
    auto world=std::make_shared<World>(&table);
    Cache frame1,frame2;Probe probe;
    require(probe.resolve(frame1,world.get())==nativeAdd,"first GameWorld validates");
    require(probe.resolve(frame2,world.get())==nativeAdd,"second GameWorld validates independently");
    require(probe.verifies==2,"same engine world is revalidated in every GameWorld instance");
    require(!probe.resolve(frame1,nullptr),"no engine world yields no collector");
    require(probe.resolve(frame1,world.get())==nativeAdd,"world returning after absence is revalidated");
    require(probe.verifies==3,"absence discards old success");
}
void retainedLifetimeAndAddressReuse() {
    Table table;
    unsigned destroyed{};
    alignas(World) std::byte storage[sizeof(World)];
    const auto construct=[&] {
        return std::shared_ptr<World>(std::construct_at(reinterpret_cast<World*>(storage),&table,&destroyed),
            [](World* value){std::destroy_at(value);});
    };
    Cache cache;Probe probe;
    auto first=construct();auto* address=first.get();
    require(probe.resolve(cache,address)==nativeAdd,"first object at reused address validates");
    std::weak_ptr<World> retained=first;first.reset();
    require(!retained.expired()&&destroyed==0,"cache pins world lifetime so its address cannot be reused");
    cache.reset();
    require(retained.expired()&&destroyed==1,"reset releases the world and discards validation");
    auto replacement=construct();replacement->readable=false;
    require(replacement.get()==address,"fixture reuses the exact object address and same vtable");
    require(!probe.resolve(cache,replacement.get()),"new invalid object at the same address cannot inherit success");
    require(probe.verifies==2,"replacement object receives full validation");
    replacement.reset();
    require(destroyed==1,"failed validation also pins its world until cache release");
    cache.reset();require(destroyed==2,"cache releases an invalid world normally");
}
void worldSwitchReleasesPreviousLease() {
    Table table;
    unsigned destroyed{};
    auto first=std::make_shared<World>(&table,&destroyed),second=std::make_shared<World>(&table,&destroyed);
    Cache cache;Probe probe;
    require(probe.resolve(cache,first.get())==nativeAdd,"first owned world validates");
    first.reset();
    require(destroyed==0,"first world survives while cached");
    require(probe.resolve(cache,second.get())==nativeAdd,"world switch revalidates");
    require(destroyed==1&&probe.verifies==2,"world switch releases old ownership after invalidating success");
}
}
int main() {
    oneValidationPerWorld();failedWorldAndTableAreCached();changedTableAndPickInvalidateSuccess();
    cacheStoresOnlyTheValidatedPick();newFrameAndMissingWorld();retainedLifetimeAndAddressReuse();worldSwitchReleasesPreviousLease();
    std::cout<<"Ray collector validation cache: "<<checks<<" checks passed\n";
}
