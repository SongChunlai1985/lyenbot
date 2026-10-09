cmake_minimum_required(VERSION 3.10)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
project(lyenbot)                                                                                   #有IMU才能启动

add_executable(lyenbot ./main.cpp)

# set(THIRDPARTY_DIR /home/lyenbot/depends)         #aarch_64
set(THIRDPARTY_DIR /home/lyenbot/depends_x86)

add_subdirectory(app)

target_link_libraries(lyenbot app)
