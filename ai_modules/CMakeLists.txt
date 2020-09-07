cmake_minimum_required(VERSION 3.12)

#project setting
project(ai_algorithm)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR})
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR})
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR})
SET(CMAKE_INCLUDE_SYSTEM_FLAG_C "-I")
SET( CMAKE_INCLUDE_SYSTEM_FLAG_CXX "-I" CACHE STRING "" FORCE )
SET( CMAKE_INCLUDE_SYSTEM_FLAG_CUDA "-I" CACHE STRING "" FORCE )
add_definitions(-std=c++11)

#include dependencies
include(cmake/ai_dependencies.cmake)

#include_directories
include_directories(${PROJECT_BINARY_DIR})
message(STATUS "PROJECT_BINARY_DIR: ${PROJECT_BINARY_DIR}")

#build imgproc_c
add_subdirectory(imgproc_c)

#build test_imgproc
add_subdirectory(imgproc_c/testing)

#build ldws_detector
add_subdirectory(ldws)

