cmake_minimum_required(VERSION 3.10)

add_library(lyn_joint lyn_joint.cpp)

target_include_directories(lyn_joint PUBLIC .)

target_link_libraries(lyn_joint PUBLIC
    lyn_info
    lyn_log
    lyn_meta
    )
