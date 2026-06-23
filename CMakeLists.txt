cmake_minimum_required(VERSION 3.25)
project(lidar_camera_calibration)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_subdirectory(src)
add_subdirectory(apps)
add_subdirectory(tests)

if(EXISTS ${CMAKE_SOURCE_DIR}/res)
    file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/res)
    file(COPY ${CMAKE_SOURCE_DIR}/res/ DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/res/)
endif()
