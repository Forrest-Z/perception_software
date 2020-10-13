cmake_minimum_required(VERSION 3.12)

#project setting
project(drivers LANGUAGES C CXX CUDA)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR})
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR})
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR})
SET(CMAKE_INCLUDE_SYSTEM_FLAG_C "-I")
SET( CMAKE_INCLUDE_SYSTEM_FLAG_CXX "-I" CACHE STRING "" FORCE )
SET( CMAKE_INCLUDE_SYSTEM_FLAG_CUDA "-I" CACHE STRING "" FORCE )

#include dependencies
include(cmake/cyber_dependencies.cmake)
include(cmake/driver_dependencies.cmake)

#include_directories
include_directories(${PROJECT_BINARY_DIR})
message(STATUS "PROJECT_BINARY_DIR: ${PROJECT_BINARY_DIR}")

#build proto
add_subdirectory(proto)

#build pandar
add_subdirectory(lidar/pandar)

#build velodyne
add_subdirectory(lidar/velodyne)

#build pandar
add_subdirectory(camera)

#build visualizer
add_subdirectory(tools/visualizer)


install(DIRECTORY "${PROJECT_BINARY_DIR}/proto" DESTINATION include ${CMAKE_INSTALL_INCLUDEDIR} FILES_MATCHING PATTERN "*.h")