#pragma once
#include "RuntimePolicy.h"

namespace fc::runtime {
inline bool supported() {return supported(REL::Module::get().version().pack());}
inline bool isSE() {return REL::Module::get().version()==REL::Version(1,5,97,0);}
}
