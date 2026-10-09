cmake_minimum_required(VERSION 3.10)

add_library(walk
    walk_main.cpp
    Matrix.cpp
    MotionStatus.cpp
    Point.cpp
    Vector.cpp
    Walking.cpp
    hip_tilt_axis_IK.cpp
    )

target_include_directories(walk PUBLIC . )

target_link_libraries(walk PUBLIC
    lyn_info
    lyn_log
    lyn_can
    lyn_meta
    )
