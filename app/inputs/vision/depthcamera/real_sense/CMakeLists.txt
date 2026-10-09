cmake_minimum_required(VERSION 3.10)

add_library(real_sense STATIC real_sense.cpp)

target_include_directories(real_sense PUBLIC
    ${THIRDPARTY_DIR}/real_sense2/include
    .
    )
target_link_libraries(real_sense PUBLIC
    ${THIRDPARTY_DIR}/real_sense2/lib/librealsense2.so
    lyn_info
    lyn_log
)
