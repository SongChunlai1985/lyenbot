cmake_minimum_required(VERSION 3.10)

add_library(outputs outputs.cpp)

add_subdirectory(lyn_gl)
add_subdirectory(axis_angle)

target_include_directories(outputs PUBLIC .)

target_link_libraries(outputs PUBLIC
    axis_angle
    lyn_gl
    )
