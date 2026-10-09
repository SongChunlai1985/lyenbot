cmake_minimum_required(VERSION 3.10)

add_library(axis_angle axis_angle.cpp calculate_ankle_motor_joint.cpp)

target_include_directories(axis_angle PUBLIC .)

target_link_libraries(axis_angle PUBLIC lyn_info lyn_log lyn_can lyn_meta)
