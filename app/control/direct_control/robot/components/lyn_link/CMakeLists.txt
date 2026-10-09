cmake_minimum_required(VERSION 3.10)

add_library(lyn_link lyn_link.cpp)

target_include_directories(lyn_link PUBLIC .)

target_link_libraries(lyn_link PUBLIC
    lyn_info
    lyn_log
    lyn_meta
    )
