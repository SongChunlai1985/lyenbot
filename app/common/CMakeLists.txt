cmake_minimum_required(VERSION 3.10)

add_library(common common.cpp)

add_subdirectory(lyn_log)
add_subdirectory(lyn_info)
add_subdirectory(lyn_meta)
add_subdirectory(lyn_usbtty)
add_subdirectory(lyn_can)

target_include_directories(common PUBLIC .)

target_link_libraries(common PUBLIC
    lyn_log
    lyn_info
    lyn_meta
    lyn_usbtty
    lyn_can
)
