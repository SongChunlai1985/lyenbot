cmake_minimum_required(VERSION 3.10)

add_library(components components.cpp)

add_subdirectory(lyn_joint)
add_subdirectory(lyn_link)

target_include_directories(components PUBLIC .)

target_link_libraries(components PUBLIC
    lyn_info
    lyn_log
    lyn_meta
    lyn_joint
    lyn_link
    )
