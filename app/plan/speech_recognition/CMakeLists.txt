cmake_minimum_required(VERSION 3.10)

add_library(speech_recognition speech_recognition.cpp)

target_include_directories(speech_recognition PUBLIC .)

target_link_libraries(speech_recognition PUBLIC
    lyn_info
    lyn_gl
    lyn_log
    )
