cmake_minimum_required(VERSION 3.12)

#project setting
project(ai_algorithm LANGUAGES C CXX)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR})
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR})
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR})
SET(CMAKE_INCLUDE_SYSTEM_FLAG_C "-I")
SET( CMAKE_INCLUDE_SYSTEM_FLAG_CXX "-I" CACHE STRING "" FORCE )
SET( CMAKE_INCLUDE_SYSTEM_FLAG_CUDA "-I" CACHE STRING "" FORCE )

#include dependencies
include(cmake/gpu_dependencies.cmake)
include(cmake/cyber_dependencies.cmake)
include(cmake/ai_dependencies.cmake)

#include_directories
include_directories(${PROJECT_BINARY_DIR})
message(STATUS "PROJECT_BINARY_DIR: ${PROJECT_BINARY_DIR}")

#build proto
add_subdirectory(proto)

#build imgproc_c
add_subdirectory(imgproc_c)

#build test_imgproc
add_subdirectory(imgproc_c/testing)

#build ldws_detector
add_subdirectory(ldws)

#build image_bgs
add_subdirectory(image_bgs)

#build lidar_common
add_subdirectory(lidar_common)

#build lidar_detection
add_subdirectory(lidar_detection)

#build lidar_ground
add_subdirectory(lidar_ground)

#build lidar_segment
add_subdirectory(lidar_segment)

#build lidar_tracking
add_subdirectory(lidar_tracking)

#build show_tools
add_subdirectory(show_tools)

#build utility
add_subdirectory(utility)

install(DIRECTORY "${PROJECT_BINARY_DIR}/proto" DESTINATION include ${CMAKE_INSTALL_INCLUDEDIR} FILES_MATCHING PATTERN "*.h")


