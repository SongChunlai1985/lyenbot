cmake_minimum_required(VERSION 3.10)

add_library(lyn_meta lyn_meta.cpp)

set(OpenCV_DIR /home/lyenbot/depends_x86/usr/share/opencv4)
find_package(OpenCV REQUIRED)

target_include_directories(lyn_meta PUBLIC .
#    ${THIRDPARTY_DIR}/usr/include/opencv4
#    ${THIRDPARTY_DIR}/usr/include/opencv4/opencv2
    ${OpenCV_INCLUDE_DIRS}
    ${THIRDPARTY_DIR}/usr/include
    )

#target_link_libraries(lyn_meta PUBLIC
#    ${THIRDPARTY_DIR}/usr/lib/aarch64-linux-gnu/libopencv_core.so.4.5.4
#    ${THIRDPARTY_DIR}/usr/lib/aarch64-linux-gnu/libopencv_face.so.4.2.0
#    ${THIRDPARTY_DIR}/usr/lib/aarch64-linux-gnu/libopencv_dnn.so.4.5.4
#    ${THIRDPARTY_DIR}/usr/lib/aarch64-linux-gnu/libopencv_imgcodecs.so.4.5.4
#    ${THIRDPARTY_DIR}/usr/lib/aarch64-linux-gnu/libopencv_imgproc.so.4.5.4
#    ${THIRDPARTY_DIR}/usr/lib/aarch64-linux-gnu/libopencv_objdetect.so.4.2.0
#    ${THIRDPARTY_DIR}/usr/lib/aarch64-linux-gnu/libopencv_calib3d.so.4.2.0
#    ${THIRDPARTY_DIR}/usr/lib/aarch64-linux-gnu/libopencv_core.so.4.2.0
#    ${THIRDPARTY_DIR}/usr/lib/aarch64-linux-gnu/libopencv_imgproc.so.4.2.0
#    ${THIRDPARTY_DIR}/usr/lib/aarch64-linux-gnu/libopencv_features2d.so.4.2.0
#    ${THIRDPARTY_DIR}/usr/lib/aarch64-linux-gnu/libopencv_flann.so.4.2.0
#    ${THIRDPARTY_DIR}/usr/lib/aarch64-linux-gnu/libopencv_highgui.so.4.5.4
#    ${THIRDPARTY_DIR}/usr/lib/aarch64-linux-gnu/libgtk-x11-2.0.so.0
#    ${THIRDPARTY_DIR}/usr/lib/aarch64-linux-gnu/libusb-1.0.so
#    ${THIRDPARTY_DIR}/usr/lib/aarch64-linux-gnu/libgdk-x11-2.0.so.0
#    ${THIRDPARTY_DIR}/usr/lib/aarch64-linux-gnu/libgdk_pixbuf-2.0.so.0
#    ${THIRDPARTY_DIR}/usr/lib/aarch64-linux-gnu/libz.so
#    ${THIRDPARTY_DIR}/usr/lib/aarch64-linux-gnu/libjpeg.so
#    ${THIRDPARTY_DIR}/usr/lib/aarch64-linux-gnu/libGL.so
#    ${THIRDPARTY_DIR}/usr/lib/aarch64-linux-gnu/libGLX.so
#    ${THIRDPARTY_DIR}/usr/lib/aarch64-linux-gnu/libX11.so
#    ${THIRDPARTY_DIR}/usr/lib/aarch64-linux-gnu/libpng16.so
#    ${THIRDPARTY_DIR}/usr/lib/aarch64-linux-gnu/libtiff.so
#    ${THIRDPARTY_DIR}/usr/lib/aarch64-linux-gnu/libGLdispatch.so
#    ${THIRDPARTY_DIR}/usr/lib/aarch64-linux-gnu/libxcb.so
#    ${THIRDPARTY_DIR}/usr/lib/aarch64-linux-gnu/libwebp.so.6
#    ${THIRDPARTY_DIR}/usr/lib/aarch64-linux-gnu/libzstd.so.1
#    ${THIRDPARTY_DIR}/usr/lib/aarch64-linux-gnu/liblzma.so.5
#    ${THIRDPARTY_DIR}/usr/lib/aarch64-linux-gnu/libjbig.so.0
#    ${THIRDPARTY_DIR}/usr/lib/aarch64-linux-gnu/libXau.so.6
#    ${THIRDPARTY_DIR}/usr/lib/aarch64-linux-gnu/libXdmcp.so.6
#    ${THIRDPARTY_DIR}/usr/lib/aarch64-linux-gnu/libbsd.so.0
#    ${THIRDPARTY_DIR}/usr/lib/aarch64-linux-gnu/libtbb.so
#    ${THIRDPARTY_DIR}/usr/lib/aarch64-linux-gnu/libcairo.so.2
#    ${THIRDPARTY_DIR}/usr/lib/aarch64-linux-gnu/libgobject-2.0.so
#    ${THIRDPARTY_DIR}/usr/lib/aarch64-linux-gnu/libglib-2.0.so
#    ${THIRDPARTY_DIR}/usr/lib/aarch64-linux-gnu/libgmodule-2.0.so
#    ${THIRDPARTY_DIR}/usr/lib/aarch64-linux-gnu/libpangocairo-1.0.so.0
#    ${THIRDPARTY_DIR}/usr/lib/aarch64-linux-gnu/libXfixes.so
#    ${THIRDPARTY_DIR}/usr/lib/aarch64-linux-gnu/libatk-1.0.so.0
#    ${THIRDPARTY_DIR}/usr/lib/aarch64-linux-gnu/libgio-2.0.so
#    ${THIRDPARTY_DIR}/usr/lib/aarch64-linux-gnu/libpangoft2-1.0.so.0
#    ${THIRDPARTY_DIR}/usr/lib/aarch64-linux-gnu/libpango-1.0.so.0
#    ${THIRDPARTY_DIR}/usr/lib/aarch64-linux-gnu/libfontconfig.so.1
#    ${THIRDPARTY_DIR}/usr/lib/aarch64-linux-gnu/libudev.so.1
#    ${THIRDPARTY_DIR}/usr/lib/aarch64-linux-gnu/libXrender.so
#    ${THIRDPARTY_DIR}/usr/lib/aarch64-linux-gnu/libXinerama.so
#    ${THIRDPARTY_DIR}/usr/lib/aarch64-linux-gnu/libXi.so.6
#    ${THIRDPARTY_DIR}/usr/lib/aarch64-linux-gnu/libXrandr.so
#    ${THIRDPARTY_DIR}/usr/lib/aarch64-linux-gnu/libXcursor.so
#    ${THIRDPARTY_DIR}/usr/lib/aarch64-linux-gnu/libXcomposite.so.1
#    ${THIRDPARTY_DIR}/usr/lib/aarch64-linux-gnu/libXdamage.so.1
#    ${THIRDPARTY_DIR}/usr/lib/aarch64-linux-gnu/libXext.so.6
#    ${THIRDPARTY_DIR}/usr/lib/aarch64-linux-gnu/libpixman-1.so.0
#    ${THIRDPARTY_DIR}/usr/lib/aarch64-linux-gnu/libfreetype.so.6
#    ${THIRDPARTY_DIR}/usr/lib/aarch64-linux-gnu/libxcb-shm.so.0
#    ${THIRDPARTY_DIR}/usr/lib/aarch64-linux-gnu/libxcb-render.so.0
#    ${THIRDPARTY_DIR}/usr/lib/aarch64-linux-gnu/libffi.so.7
#    ${THIRDPARTY_DIR}/usr/lib/aarch64-linux-gnu/libpcre.so.3
#    ${THIRDPARTY_DIR}/usr/lib/aarch64-linux-gnu/libmount.so.1
#    ${THIRDPARTY_DIR}/usr/lib/aarch64-linux-gnu/libselinux.so.1
#    ${THIRDPARTY_DIR}/usr/lib/aarch64-linux-gnu/libharfbuzz.so.0
#    ${THIRDPARTY_DIR}/usr/lib/aarch64-linux-gnu/libfribidi.so.0
#    ${THIRDPARTY_DIR}/usr/lib/aarch64-linux-gnu/libthai.so.0
#    ${THIRDPARTY_DIR}/usr/lib/aarch64-linux-gnu/libexpat.so.1
#    ${THIRDPARTY_DIR}/usr/lib/aarch64-linux-gnu/libuuid.so.1
#    ${THIRDPARTY_DIR}/usr/lib/aarch64-linux-gnu/libblkid.so.1
#    ${THIRDPARTY_DIR}/usr/lib/aarch64-linux-gnu/libpcre2-8.so.0
#    ${THIRDPARTY_DIR}/usr/lib/aarch64-linux-gnu/libgraphite2.so.3
#    ${THIRDPARTY_DIR}/usr/lib/aarch64-linux-gnu/libdatrie.so.1
#    )                                                            # aarch64

target_link_libraries(lyn_meta PUBLIC
    ${OpenCV_LIBS}
    )
