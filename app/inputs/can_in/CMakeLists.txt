cmake_minimum_required(VERSION 3.10)

add_library(can_in can_in.cpp)

target_include_directories(can_in PUBLIC .)

target_link_libraries(can_in PUBLIC
    lyn_log
    lyn_info
    lyn_meta
    lyn_can
)




