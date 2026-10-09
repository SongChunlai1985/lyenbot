 cmake_minimum_required(VERSION 3.10)

add_library(plan plan.cpp)

add_subdirectory(reconstruction)
add_subdirectory(avoiding)
# add_subdirectory(face_recognition)
# add_subdirectory(object_detection)
# add_subdirectory(speech_recognition)

target_include_directories(plan PUBLIC .)

target_link_libraries(plan PUBLIC
    reconstruction
    avoiding
#    face_recognition
#    object_detection
#    speech_recognition
)
