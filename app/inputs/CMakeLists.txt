cmake_minimum_required(VERSION 3.10)

add_library(inputs inputs.cpp)

# add_subdirectory(vision)
add_subdirectory(imu)
add_subdirectory(can_in)

target_include_directories(inputs PUBLIC .)

target_link_libraries(inputs PUBLIC
#    vision
    imu
    can_in
)
