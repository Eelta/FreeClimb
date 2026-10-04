#pragma once
#include <atomic>
#include <mutex>
#include <type_traits>

namespace fc {
template<class Fn> class HookPublication {
    static_assert(std::is_pointer_v<Fn>&&std::is_function_v<std::remove_pointer_t<Fn>>);
    std::atomic<Fn> original{};
    std::mutex mutex;
    Fn installedReplacement{};
    bool installed{};
public:
    Fn get() const {return original.load(std::memory_order_acquire);}
    template<class Publish> bool install(Fn expected,Fn replacement,Publish&& publish) {
        std::scoped_lock lock(mutex);
        if(installed)return installedReplacement==replacement;
        if(!expected||!replacement||expected==replacement)return false;
        original.store(expected,std::memory_order_release);
        if(!publish(expected,replacement)) {
            original.store(nullptr,std::memory_order_release);return false;
        }
        installedReplacement=replacement;installed=true;return true;
    }
};
}
