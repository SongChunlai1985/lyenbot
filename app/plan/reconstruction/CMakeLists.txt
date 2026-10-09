cmake_minimum_required(VERSION 3.10)

add_library(reconstruction reconstruction.cpp)
add_subdirectory(kalman_filter)
add_subdirectory(imu_complementary_filter)
add_subdirectory(imu_filter_madgwick)

target_include_directories(reconstruction PUBLIC .)

target_link_libraries(reconstruction PUBLIC
    lyn_log
    lyn_info
    lyn_meta
    kalman_filter
    imu_complementary_filter
    imu_filter_madgwick
    )

