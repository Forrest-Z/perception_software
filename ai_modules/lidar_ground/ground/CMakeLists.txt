project(lidar_ground)

#include ai_algorithm directories
include_directories(${ai_algorithm_BINARY_DIR})
include_directories(${ai_algorithm_SOURCE_DIR})

#collect source files
file(GLOB LIDAR_GROUND_SRCS 
    "${PROJECT_SOURCE_DIR}/*.cc"
	)

#build imgproc_c library
add_library(lidar_ground SHARED ${LIDAR_GROUND_SRCS})
target_link_libraries(lidar_ground
    lidar_common
    boost_system
    pcl_common
    pcl_io
    pcl_filters
    pcl_sample_consensus
    pcl_segmentation
)

install(DIRECTORY "${PROJECT_SOURCE_DIR}" DESTINATION include ${CMAKE_INSTALL_INCLUDEDIR} FILES_MATCHING PATTERN "*.h")
install(FILES "${ai_algorithm_BINARY_DIR}/liblidar_ground.so" DESTINATION lib ${CMAKE_INSTALL_LIBDIR})