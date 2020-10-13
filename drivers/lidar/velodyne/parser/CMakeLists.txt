project(velodyne_convert_component)

#include drivers directories
include_directories(${drivers_BINARY_DIR})
include_directories(${drivers_SOURCE_DIR})

#collect source files
file(GLOB COMMON_SRCS
    "${drivers_SOURCE_DIR}/common/configs/*.cc"
    "${drivers_SOURCE_DIR}/common/time/*.cc"
    "${drivers_SOURCE_DIR}/common/util/*.cc"
	)

#collect source files
file(GLOB VELODYNE_SRCS 
    "${PROJECT_SOURCE_DIR}/*.cc"
)

#build pandar_sdk library
add_library(velodyne_convert_component SHARED ${VELODYNE_SRCS} ${COMMON_SRCS})
target_link_libraries(velodyne_convert_component 
    drivers_proto
    cyber 
    cyber_proto
    gflags
    yaml-cpp
)

install(FILES "${drivers_BINARY_DIR}/libvelodyne_convert_component.so" DESTINATION lib ${CMAKE_INSTALL_LIBDIR})