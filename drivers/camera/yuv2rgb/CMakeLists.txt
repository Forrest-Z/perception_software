cmake_minimum_required(VERSION 3.8 FATAL_ERROR)
project(yuv2rgb)

#include drivers directories
include_directories(${drivers_BINARY_DIR})
include_directories(${drivers_SOURCE_DIR})

SET(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS};-gencode arch=compute_52,code=sm_52)

#set(CUDA_TOOLKIT_ROOT_DIR /usr/local/cuda)
#enable_language(CUDA)
find_package(CUDA REQUIRED)
message("CUDA_INCLUDE_DIRS : " ${CUDA_INCLUDE_DIRS})
include_directories(${CUDA_INCLUDE_DIRS})

file(GLOB_RECURSE YUV2RGB_HEADERS  ${CMAKE_CURRENT_LIST_DIR}/*.h ${CMAKE_CURRENT_LIST_DIR}/*.cuh)
file(GLOB YUV2RGB_SOURCES  ${CMAKE_CURRENT_LIST_DIR}/*.cpp ${CMAKE_CURRENT_LIST_DIR}/*.cu)

add_library(yuv2rgb SHARED ${YUV2RGB_SOURCES} ${YUV2RGB_HEADERS})

# Request that yuv2rgb be built with -std=c++11
# As this is a public compile feature anything that links to yuv2rgb
# will also build with -std=c++11
target_compile_features(yuv2rgb PUBLIC cxx_std_11)

# We need to explicitly state that we need all CUDA files in the yuv2rgb
# library to be built with -dc as the member functions could be called by
# other libraries and executables
set_target_properties(yuv2rgb PROPERTIES POSITION_INDEPENDENT_CODE ON)
set_target_properties(yuv2rgb PROPERTIES CUDA_SEPARABLE_COMPILATION ON)

install(FILES "${drivers_BINARY_DIR}/libyuv2rgb.so" DESTINATION lib ${CMAKE_INSTALL_LIBDIR})