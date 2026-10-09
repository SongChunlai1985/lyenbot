cmake_minimum_required(VERSION 3.10)

add_library(mid360 mid360.cpp)

target_include_directories(mid360 PUBLIC
    ${THIRDPARTY_DIR}/usr/include
    .
    )
target_link_libraries(mid360 PUBLIC
    ${THIRDPARTY_DIR}/usr/lib/liblivox_lidar_sdk_shared.so
    lyn_log
    lyn_info
)
