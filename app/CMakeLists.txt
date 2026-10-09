cmake_minimum_required(VERSION 3.10)

add_library(app app.cpp)

add_subdirectory(common)
add_subdirectory(inputs)
add_subdirectory(plan)
add_subdirectory(control)
add_subdirectory(outputs)

target_include_directories(app PUBLIC .)

target_link_libraries(app PUBLIC
    common
    inputs
    plan
    control
    outputs
    )
