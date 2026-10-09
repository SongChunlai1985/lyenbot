cmake_minimum_required(VERSION 3.10)

add_library(control control.cpp)

#add_subdirectory(onnx_inference)
add_subdirectory(direct_control)

target_include_directories(control PUBLIC .)

target_link_libraries(control PUBLIC
#    onnx_inference
    direct_control
    lyn_log
)
