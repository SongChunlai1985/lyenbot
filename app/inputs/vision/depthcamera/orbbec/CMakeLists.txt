cmake_minimum_required(VERSION 3.10)

add_library(orbbec STATIC orbbec.cpp)

target_include_directories(orbbec PUBLIC
    ${THIRDPARTY_DIR}/OrbbecSDK_v1.10.22/SDK/include
    .
    )
target_link_libraries(orbbec PUBLIC
    lyn_info
    lyn_log
    ${THIRDPARTY_DIR}/OrbbecSDK_v1.10.22/SDK/lib/libOrbbecSDK.so
    pthread
)
