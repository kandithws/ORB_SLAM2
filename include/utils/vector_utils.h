//
// Created by kandithws on 17/4/2562.
//

#ifndef ORB_SLAM2_VECTOR_UTILS_H
#define ORB_SLAM2_VECTOR_UTILS_H

#include <sstream>
#include <vector>

namespace ORB_SLAM2 {
namespace utils {

template <typename T>
static std::string VectorToString(std::vector<T> vect, std::string delim=","){
    std::stringstream ss;
    ss << "{ ";
    for (auto it = vect.cbegin(); it != vect.cend(); it++){
        ss << *it << delim;
    }
    ss << " }";

    return ss.str();
}

}
}

#endif //ORB_SLAM2_VECTOR_UTILS_H
