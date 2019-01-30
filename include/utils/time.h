//
// Created by kandithws on 26/1/2562.
//

#ifndef ORB_SLAM2_TIME_H
#define ORB_SLAM2_TIME_H

#include <chrono>

namespace ORB_SLAM2 {
namespace utils {
namespace time {
typedef std::chrono::time_point<std::chrono::system_clock> time_point;

inline time_point time_now() { return std::chrono::system_clock::now(); }

template<typename T>
inline int time_diff(const time_point &t1, const time_point &t2) {
    return std::chrono::duration_cast<T>(t2 - t1).count();
}

// seconds
inline float time_diff_second(const time_point &t1, const time_point &t2) {
    return std::chrono::duration<float>(t2 - t1).count();
}

template<typename T>
inline int time_diff_from_now(const time_point &t) {
    return std::chrono::duration_cast<T>(std::chrono::system_clock::now() - t).count();
}

// seconds
inline float time_diff_from_now_second(const time_point &t1) {
    return std::chrono::duration<float>(std::chrono::system_clock::now() - t1).count();
}

}
}
}
#endif //ORB_SLAM2_TIME_H
