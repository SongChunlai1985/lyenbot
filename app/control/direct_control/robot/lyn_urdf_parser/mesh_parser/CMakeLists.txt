cmake_minimum_required(VERSION 3.10)

add_library(mesh_parser mesh_parser.cpp)

target_include_directories(mesh_parser PUBLIC .)

target_link_libraries(mesh_parser PUBLIC
    lyn_info
    lyn_log
    lyn_meta
    )
