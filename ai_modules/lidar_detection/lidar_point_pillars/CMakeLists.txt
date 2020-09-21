project(lidar_point_pillars)

#include ai_algorithm directories
include_directories(${ai_algorithm_BINARY_DIR})
include_directories(${ai_algorithm_SOURCE_DIR})

file(GLOB_RECURSE POINT_PILLARS_HEADERS  ${CMAKE_CURRENT_LIST_DIR}/*.h ${CMAKE_CURRENT_LIST_DIR}/*.cuh)
file(GLOB POINT_PILLARS_SOURCES  ${CMAKE_CURRENT_LIST_DIR}/*.cc ${CMAKE_CURRENT_LIST_DIR}/*.cu)

add_library(lidar_point_pillars SHARED ${POINT_PILLARS_SOURCES} ${POINT_PILLARS_HEADERS})
target_link_libraries(lidar_point_pillars
                      cuda
                      cublas
                      cudart
                      cudnn
                      nvinfer
                      nvparsers
                      nvinfer_plugin
                      nvcaffe_parser
                      nvonnxparser
                    )

# Request that lidar_point_pillars be built with -std=c++11
# As this is a public compile feature anything that links to yulidar_point_pillarsv2rgb
# will also build with -std=c++11
target_compile_features(lidar_point_pillars PUBLIC cxx_std_11)

# We need to explicitly state that we need all CUDA files in the lidar_point_pillars
# library to be built with -dc as the member functions could be called by
# other libraries and executables
set_target_properties(lidar_point_pillars PROPERTIES POSITION_INDEPENDENT_CODE ON)
set_target_properties(lidar_point_pillars PROPERTIES CUDA_SEPARABLE_COMPILATION ON)

install(DIRECTORY "${ai_algorithm_SOURCE_DIR}/lidar_detection/lidar_point_pillars" DESTINATION include ${CMAKE_INSTALL_INCLUDEDIR} FILES_MATCHING PATTERN "*.h")
install(FILES "${ai_algorithm_BINARY_DIR}/liblidar_point_pillars.so" DESTINATION lib ${CMAKE_INSTALL_LIBDIR})
