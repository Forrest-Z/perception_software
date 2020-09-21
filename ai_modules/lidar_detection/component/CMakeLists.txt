project(lidar_detection_component)

#include ai_algorithm directories
include_directories(${ai_algorithm_BINARY_DIR})
include_directories(${ai_algorithm_SOURCE_DIR})
message(STATUS "ai_algorithm_SOURCE_DIR: ${ai_algorithm_SOURCE_DIR}")

#collect source files
file(GLOB COMMON_SRCS
    "${ai_algorithm_SOURCE_DIR}/base/*.cc"
    "${ai_algorithm_SOURCE_DIR}/common/*.cc"
	)

file(GLOB APP_SRCS
    "${ai_algorithm_SOURCE_DIR}/lidar_detection/app/*.cc"
	)

#build lidar_detection_component library
add_library(lidar_detection_component SHARED lidar_detection_component.cc ${APP_SRCS} ${COMMON_SRCS})
target_link_libraries(lidar_detection_component 
    lidar_point_pillars
    ai_proto
    cyber 
    cyber_proto
    gflags
    glog
    opencv_core
    opencv_imgcodecs 
    opencv_imgproc
    pcl_common
    pcl_io
)

install(FILES "${ai_algorithm_BINARY_DIR}/liblidar_detection_component.so" DESTINATION lib ${CMAKE_INSTALL_LIBDIR})
