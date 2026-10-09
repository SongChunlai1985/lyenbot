cmake_minimum_required(VERSION 3.10)

add_library(imu_complementary_filter imu_complementary_filter.cpp)

target_include_directories(imu_complementary_filter PUBLIC .)

target_link_libraries(imu_complementary_filter PUBLIC
    lyn_info
    lyn_meta
    )

