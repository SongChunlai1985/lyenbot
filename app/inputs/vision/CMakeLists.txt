cmake_minimum_required(VERSION 3.10)

add_library(vision vision.cpp)

add_subdirectory(depthcamera)
add_subdirectory(lidar)

target_include_directories(vision PUBLIC .)

target_link_libraries(vision PUBLIC
    depthcamera
    mid360
    lyn_log
    lyn_info
    lyn_meta
)




