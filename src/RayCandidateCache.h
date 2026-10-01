#pragma once
#include <array>
#include <cstdint>

namespace fc {
template<class Value> class RayCandidateCache {
    struct Entry {const void* key{};Value value{};};
    std::array<Entry,16> entries{};
public:
    template<class Compute> const Value& resolve(const void* key,Compute compute) {
        const auto address=reinterpret_cast<std::uintptr_t>(key);
        auto& entry=entries[((address>>4)^(address>>11))&(entries.size()-1)];
        if(entry.key!=key){entry.value=compute();entry.key=key;}
        return entry.value;
    }
};
}
