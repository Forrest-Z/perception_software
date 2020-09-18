project(lidar_detection)

#include ai_algorithm directories
include_directories(${ai_algorithm_BINARY_DIR})
include_directories(${ai_algorithm_SOURCE_DIR})

add_subdirectory(lidar_point_pillars)

add_subdirectory(component)

add_subdirectory(tools)