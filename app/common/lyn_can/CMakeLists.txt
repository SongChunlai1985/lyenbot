cmake_minimum_required(VERSION 3.10)

add_library(lyn_can lyn_can.cpp)

target_include_directories(lyn_can PUBLIC . )
target_link_libraries(lyn_can PUBLIC jsoncpp )
