cmake_minimum_required(VERSION 3.10)

add_library(face_recognition face_recognition.cpp)

target_include_directories(face_recognition PUBLIC .)

target_link_libraries(face_recognition PUBLIC
    lyn_info
    lyn_gl
    lyn_log
    )
