project(lidar_common)

#include ai_algorithm directories
include_directories(${ai_algorithm_BINARY_DIR})
include_directories(${ai_algorithm_SOURCE_DIR})

#collect source files
file(GLOB COMMON_SRCS
    "${ai_algorithm_SOURCE_DIR}/base/*.cc"
    "${ai_algorithm_SOURCE_DIR}/common/*.cc"
    "${ai_algorithm_SOURCE_DIR}/common/geometry/*.cc"
    "${ai_algorithm_SOURCE_DIR}/common/lidar_processing/*.cc"
)

#collect source files
file(GLOB LIDAR_COMMON_SRCS 
    "${PROJECT_SOURCE_DIR}/*.cc"
	)

#build imgproc_c library
add_library(lidar_common SHARED ${LIDAR_COMMON_SRCS} ${COMMON_SRCS})
target_link_libraries(lidar_common
    cyber
    gflags
    glog
    opencv_core
    opencv_imgcodecs 
    opencv_imgproc
    boost_system
    pcl_common
    pcl_io
    pcl_filters
    pcl_sample_consensus
    pcl_keypoints
)

install(DIRECTORY "${PROJECT_SOURCE_DIR}" DESTINATION include ${CMAKE_INSTALL_INCLUDEDIR} FILES_MATCHING PATTERN "*.h")
install(FILES "${ai_algorithm_BINARY_DIR}/liblidar_common.so" DESTINATION lib ${CMAKE_INSTALL_LIBDIR})