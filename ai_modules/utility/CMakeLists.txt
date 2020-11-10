project(ai_utility)

#include ai_algorithm directories
include_directories(${ai_algorithm_BINARY_DIR})
include_directories(${ai_algorithm_SOURCE_DIR})

add_subdirectory(registerer)

add_subdirectory(thread)