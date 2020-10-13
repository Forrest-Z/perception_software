project(velodyne)

#include drivers directories
include_directories(${drivers_BINARY_DIR})
include_directories(${drivers_SOURCE_DIR})

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")

#build driver
add_subdirectory(driver)

#build parser
add_subdirectory(parser)