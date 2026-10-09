cmake_minimum_required(VERSION 3.10)

add_library(object_detection object_detection.cpp)

target_include_directories(object_detection PUBLIC .)

target_link_libraries(object_detection PUBLIC
    lyn_info
    lyn_gl
    lyn_log
    )
