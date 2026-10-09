cmake_minimum_required(VERSION 3.10)

add_library(depthcamera STATIC depthcamera.cpp)

add_subdirectory(orbbec)
add_subdirectory(real_sense)

target_include_directories(depthcamera PUBLIC .)

target_link_libraries(depthcamera PUBLIC
    lyn_info
    orbbec
    real_sense
)
