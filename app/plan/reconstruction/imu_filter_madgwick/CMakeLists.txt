cmake_minimum_required(VERSION 3.10)

add_library(imu_filter_madgwick imu_filter_madgwick.cpp)

target_include_directories(imu_filter_madgwick PUBLIC .)

target_link_libraries(imu_filter_madgwick PUBLIC
    lyn_info
    lyn_meta
    )

