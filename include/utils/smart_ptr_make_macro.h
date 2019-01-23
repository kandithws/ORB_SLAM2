//
// Created by kandithws on 23/1/2562.
//

#ifndef ORB_SLAM2_SMART_PTR_MAKE_MACRO_H
#define ORB_SLAM2_SMART_PTR_MAKE_MACRO_H

/*
 * @brief Macros for new smart_ptr. 
 * Do not use make_shared! it will make 32-bit system memory alignment fail
 * when working with  "EIGEN_MAKE_ALIGNED_OPERATOR_NEW"
 * i.e. all objects "pcl" libs
 */

#define STD_MAKE_SHARED(T, ...) std::shared_ptr<T>(new T(__VA_ARGS__))
#define BOOST_MAKE_SHARED(T, ...) boost::shared_ptr<T>(new T(__VA_ARGS__))

#endif //ORB_SLAM2_SMART_PTR_MAKE_MACRO_H
