cmake_minimum_required(VERSION 3.10)

add_library(lyn_gl lyn_gl.cpp)

target_include_directories(lyn_gl PUBLIC .
    ${THIRDPARTY_DIR}/usr/include
    )
target_link_libraries(lyn_gl PUBLIC
#    ${THIRDPARTY_DIR}/usr/lib/aarch64-linux-gnu/libGLU.so
#    ${THIRDPARTY_DIR}/usr/lib/aarch64-linux-gnu/libGLEW.so
#    ${THIRDPARTY_DIR}/usr/lib/aarch64-linux-gnu/libglfw.so
    GL
    GLU
    GLEW
    glfw
    lyn_info
    lyn_meta
    lyn_log
    )
