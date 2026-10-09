cmake_minimum_required(VERSION 3.10)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_library(lyn_info lyn_info.cpp)

set(OpenCV_DIR /home/lyenbot/depends_x86/usr/share/opencv4)
find_package(OpenCV REQUIRED)
target_include_directories(lyn_info
    PUBLIC
#    ${THIRDPARTY_DIR}/usr/include/opencv4/opencv2
#    ${THIRDPARTY_DIR}/usr/include/opencv4
    ${OpenCV_INCLUDE_DIRS}
    .
    )
target_link_libraries(lyn_info PUBLIC
    ${OpenCV_LIBS}
    )
