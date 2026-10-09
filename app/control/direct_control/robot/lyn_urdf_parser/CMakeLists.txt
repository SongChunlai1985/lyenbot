cmake_minimum_required(VERSION 3.10)

add_library(lyn_urdf_parser lyn_urdf_parser.cpp)

add_subdirectory(mesh_parser)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

target_include_directories(lyn_urdf_parser PUBLIC
    ${THIRDPARTY_DIR}/usr/include/opencv4
    ${THIRDPARTY_DIR}/usr/include/opencv4/opencv2
    ${THIRDPARTY_DIR}/usr/include
    ${THIRDPARTY_DIR}/usr/include/urdfdom
    ${THIRDPARTY_DIR}/usr/include/urdfdom_headers
    .
    )

target_link_libraries(lyn_urdf_parser PUBLIC
    lyn_info
    lyn_log
    ${THIRDPARTY_DIR}/usr/lib/liburdfdom_model.so
    ${THIRDPARTY_DIR}/usr/lib/libtinyxml2.so
    ${THIRDPARTY_DIR}/usr/lib/libconsole_bridge.so
    mesh_parser
    components
    )
