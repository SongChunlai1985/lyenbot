cmake_minimum_required(VERSION 3.10)

add_library(direct_control direct_control.cpp)

add_subdirectory(robot)
add_subdirectory(walk)

target_include_directories(direct_control PUBLIC .)

target_link_libraries(direct_control PUBLIC
    lyn_info
    lyn_log
    lyn_can
    robot
    walk
    )
