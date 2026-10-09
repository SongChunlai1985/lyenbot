cmake_minimum_required(VERSION 3.10)

add_library(onnx_inference onnx_inference.cpp)

target_include_directories(onnx_inference PUBLIC .)

target_link_libraries(onnx_inference PUBLIC
    lyn_info
    lyn_log
    )
