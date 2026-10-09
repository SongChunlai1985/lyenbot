cmake_minimum_required(VERSION 3.10)

add_library(lyn_log lyn_log.cpp)

target_include_directories(lyn_log PUBLIC . lyn_info)
