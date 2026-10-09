cmake_minimum_required(VERSION 3.10)

add_library(avoiding avoiding.cpp)

target_include_directories(avoiding PUBLIC .)

target_link_libraries(avoiding PUBLIC
    lyn_info
    lyn_gl
    lyn_log
    )
