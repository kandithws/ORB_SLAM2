//
// Created by kandithws on 19/1/2562.
//

// #include <ros/ros.h>
#include "spdlog/spdlog.h"

int main (int argc, char** argv) {
 //ros::init(argc, argv, "test_node");
    spdlog::set_pattern("%^[%E.%F][%l][func:%!:%@] %v%$");
    spdlog::info("Welcome to spdlog version {}.{}.{} !", SPDLOG_VER_MAJOR, SPDLOG_VER_MINOR, SPDLOG_VER_PATCH);
    spdlog::warn("Easy padding in numbers like {:08d}", 12);
    spdlog::critical("Support for int: {0:d};  hex: {0:x};  oct: {0:o}; bin: {0:b}", 42);
    spdlog::info("Support for floats {:03.2f}", 1.23456);
    std::string test("HEEe");
    spdlog::info("Positional args are {1} {0}..", test, "supported");
    spdlog::info("{:>8} aligned, {:<8} aligned", "right", "left");
    SPDLOG_INFO("HELLO WORLD");
    SPDLOG_ERROR("HELLO WORLD");
}