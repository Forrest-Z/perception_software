project(show_tools)

#include ai_algorithm directories
include_directories(${ai_algorithm_BINARY_DIR})
include_directories(${ai_algorithm_SOURCE_DIR})

add_subdirectory(qt_show)

add_subdirectory(pcl_show)