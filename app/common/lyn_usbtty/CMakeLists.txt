cmake_minimum_required(VERSION 3.10)

add_library(lyn_usbtty lyn_usbtty.cpp serialib.cpp)

target_include_directories(lyn_usbtty PUBLIC .)
