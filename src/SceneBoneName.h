#pragma once
#include <string_view>

namespace fc {
inline bool sameSceneBoneName(std::string_view left,std::string_view right) {
    if(left.size()!=right.size())return false;
    for(std::size_t i=0;i<left.size();++i) {
        const auto fold=[](unsigned char value){return value>='A'&&value<='Z'?value+('a'-'A'):value;};
        if(fold(static_cast<unsigned char>(left[i]))!=fold(static_cast<unsigned char>(right[i])))return false;
    }
    return true;
}
}
