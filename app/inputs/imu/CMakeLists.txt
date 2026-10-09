cmake_minimum_required(VERSION 3.10)

add_library(imu imu.cpp)

target_include_directories(imu PUBLIC .)

target_link_libraries(imu PUBLIC
    lyn_log
    lyn_info
    lyn_meta
    lyn_usbtty
)




