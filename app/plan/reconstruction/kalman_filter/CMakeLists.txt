cmake_minimum_required(VERSION 3.10)

add_library(kalman_filter kalman_filter.cpp)

target_include_directories(kalman_filter PUBLIC .)

target_link_libraries(kalman_filter PUBLIC
    lyn_info
    lyn_meta
    )

