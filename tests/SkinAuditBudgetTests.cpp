#include "SkinAuditBudget.h"
#include "LateWorldUpdate.h"
#include <array>
#include <chrono>
#include <iostream>
#include <stdexcept>
#include <thread>
#include <vector>
using namespace fc;
static void check(bool value,const char* message){if(!value)throw std::runtime_error(message);}
static void crowdedRenderFrames() {
    SkinAuditBudget budget;std::array<bool,500> seen{};
    std::uint64_t admitted=0,inputs=0,ancestors=0,originalCalls=0;
    const auto start=std::chrono::steady_clock::now();

    for(std::uint32_t frame=0;frame<2000;++frame) {
        unsigned perFrame=0,frameInputs=0,frameAncestors=0;
        for(unsigned call=0;call<500;++call) {
            if(budget.trySample(frame)) {
                ++perFrame;++admitted;seen[call]=true;
                SkinAuditWork work;
                for(int bone=0;bone<4096&&work.input();++bone)++frameInputs;
                for(int parent=0;parent<4096&&work.ancestor();++parent)++frameAncestors;
                check(work.exhausted&&work.inputs==64&&work.ancestors==64,"oversized meshes and ancestry cannot exceed the sample work budget");
            }
            ++originalCalls;
        }
        check(perFrame<=4&&frameInputs<=256&&frameAncestors<=256,"all geometry/NPC calls share a hard global per-frame budget");
        inputs+=frameInputs;ancestors+=frameAncestors;
    }
    check(originalCalls==1000000&&admitted==8000,"one million calls retain every original invocation but admit only four observers per frame");
    for(bool visited:seen)check(visited,"rotating sample window eventually observes late geometry as well as the first NPC draws");
    const auto ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count();
    std::cout<<"1M simulated calls: admitted="<<admitted<<" inspected="<<inputs<<" ancestors="<<ancestors<<" elapsedMs="<<ms<<'\n';
}
static void concurrentAndStaleFrames() {
    SkinAuditBudget budget;std::atomic<unsigned> admitted{},originalCalls{};
    std::vector<std::thread> workers;
    for(int worker=0;worker<8;++worker)workers.emplace_back([&]{
        for(int draw=0;draw<2000;++draw){if(budget.trySample(100))++admitted;++originalCalls;}
    });
    for(auto& worker:workers)worker.join();
    check(admitted<=4&&originalCalls==16000,"render threads cannot multiply a global observer allowance or lose original calls");
    unsigned next=0;for(int i=0;i<16000;++i)next+=budget.trySample(101);
    check(next==4,"a new render frame receives one allowance");
    for(int i=0;i<20000;++i)check(!budget.trySample(100),"late older-frame work cannot reopen the previous frame budget");
    for(int i=0;i<20000;++i)check(!budget.trySample(101),"same-frame repeated draws cannot replenish exhausted observation budget");
    SkinAuditBudget wrap;
    for(auto frame:{0xfffffffeu,0xffffffffu,0u,1u}) {
        unsigned count=0;for(int draw=0;draw<500;++draw)count+=wrap.trySample(frame);
        check(count==4,"engine frame counter wrap does not disable diagnostics or amplify work");
    }
}
static void workLimitIsNotAVisualFailure() {
    struct Node {Node* parent{};int local=1;};std::array<Node,80> chain;
    for(std::size_t i=1;i<chain.size();++i)chain[i].parent=&chain[i-1];
    SkinAuditWork work;std::unordered_map<Node*,std::optional<int>> cache;
    const auto value=ownedSkinWorld<Node*,int>(&chain.back(),&chain.front(),1,
        [](Node* node){return node->parent;},[&](Node* node)->std::optional<int>{if(!work.ancestor())return {};return node->local;},
        [](int a,int b){return a+b;},cache);
    check(!value&&work.exhausted&&work.ancestors==64,"budget-limited ancestry returns an unavailable observation, not a fabricated matrix");
    unsigned mismatch=0;if(value&&*value!=80)++mismatch;
    check(mismatch==0,"an incomplete diagnostic cannot be counted as missing body output");
    check(skinInputKind(0)==SkinInputKind::body&&skinInputKind(96)==SkinInputKind::body,
        "mapped authored body tracks are classified explicitly");
    check(skinInputKind(99)==SkinInputKind::bridge&&skinInputKind(167)==SkinInputKind::bridge,
        "native intermediary controller nodes are distinct from authored body inputs");
    check(skinInputKind(std::nullopt)==SkinInputKind::extra,"unmapped physics/attachment inputs cannot inflate body mismatch counts");
    unsigned bodyErrors=0,bridgeErrors=0,extraErrors=0;
    for(auto kind:{SkinInputKind::extra,SkinInputKind::extra,SkinInputKind::extra,SkinInputKind::extra,SkinInputKind::extra}) {
        if(kind==SkinInputKind::body)++bodyErrors;else if(kind==SkinInputKind::bridge)++bridgeErrors;else ++extraErrors;
    }
    check(bodyErrors==0&&bridgeErrors==0&&extraErrors==5,"the observed 1-body/5-extra geometry can report extras without a false whole-body warning");
}
int main() {
    try {crowdedRenderFrames();concurrentAndStaleFrames();workLimitIsNotAVisualFailure();std::cout<<"PASS: bounded nonblocking skin observation and body/physics classification\n";}
    catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}
}
