#pragma once
#include "../../src/HkxSpline.h"

namespace fc {
bool decodeConverterSplineTransforms(const HkxSplineData&,std::vector<Pose>&,std::string&);
}
