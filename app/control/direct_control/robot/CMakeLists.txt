cmake_minimum_required(VERSION 3.10)

add_library(robot robot.cpp)

add_subdirectory(components)
add_subdirectory(lyn_urdf_parser)

target_include_directories(robot PUBLIC .)

target_link_libraries(robot PUBLIC
    components
    lyn_urdf_parser
    )
