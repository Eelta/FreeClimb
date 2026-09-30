#pragma once
#include "RuntimeVersion.h"
#include <cstring>
#include <limits>

namespace fc::runtime {
inline bool readable(std::uintptr_t address,std::size_t bytes) {
    if(!address||!bytes||bytes>std::numeric_limits<std::uintptr_t>::max()-address)return false;
    const auto end=address+bytes;
    while(address<end) {
        MEMORY_BASIC_INFORMATION info{};
        if(!VirtualQuery(reinterpret_cast<const void*>(address),&info,sizeof(info))||
            info.State!=MEM_COMMIT||(info.Protect&(PAGE_GUARD|PAGE_NOACCESS)))return false;
        const auto protection=info.Protect&0xFF;
        if(protection!=PAGE_READONLY&&protection!=PAGE_READWRITE&&protection!=PAGE_WRITECOPY&&
            protection!=PAGE_EXECUTE_READ&&protection!=PAGE_EXECUTE_READWRITE&&protection!=PAGE_EXECUTE_WRITECOPY)return false;
        const auto start=reinterpret_cast<std::uintptr_t>(info.BaseAddress);
        if(info.RegionSize>std::numeric_limits<std::uintptr_t>::max()-start)return false;
        const auto next=start+info.RegionSize;
        if(next<=address)return false;
        address=next;
    }
    return true;
}
inline bool callable(std::uintptr_t address) {
    MEMORY_BASIC_INFORMATION info{};
    if(!address||!VirtualQuery(reinterpret_cast<const void*>(address),&info,sizeof(info))||
        info.State!=MEM_COMMIT||(info.Protect&(PAGE_GUARD|PAGE_NOACCESS)))return false;
    const auto protection=info.Protect&0xFF;
    return protection==PAGE_EXECUTE||protection==PAGE_EXECUTE_READ||
        protection==PAGE_EXECUTE_READWRITE||protection==PAGE_EXECUTE_WRITECOPY;
}
inline bool executable(std::uintptr_t address) {
    for(const auto name:{REL::Segment::textx,REL::Segment::textw}) {
        const auto segment=REL::Module::get().segment(name);
        if(address>=segment.address()&&address-segment.address()<segment.size())return callable(address);
    }
    return false;
}
inline bool hookSite(std::uintptr_t table,std::size_t slot) {
    if(slot>0x1000||!readable(table,(slot+1)*sizeof(std::uintptr_t)))return false;
    std::uintptr_t target{};
    std::memcpy(&target,reinterpret_cast<const void*>(table+slot*sizeof(target)),sizeof(target));
    return callable(target);
}
}
