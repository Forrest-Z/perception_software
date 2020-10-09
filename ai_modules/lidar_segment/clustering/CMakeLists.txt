project(lidar_clustering)

#include ai_algorithm directories
include_directories(${ai_algorithm_BINARY_DIR})
include_directories(${ai_algorithm_SOURCE_DIR})

#collect source files
file(GLOB LIDAR_GROUND_SRCS 
    "${PROJECT_SOURCE_DIR}/*.cc"
	)

#build lidar_clustering library
add_library(lidar_clustering SHARED ${LIDAR_GROUND_SRCS})
target_link_libraries(lidar_clustering
    lidar_common
    boost_system
    pcl_common
    pcl_io
    pcl_filters
    pcl_features
    pcl_kdtree
    pcl_search
    pcl_segmentation
)

install(DIRECTORY "${PROJECT_SOURCE_DIR}" DESTINATION include ${CMAKE_INSTALL_INCLUDEDIR} FILES_MATCHING PATTERN "*.h")
install(FILES "${ai_algorithm_BINARY_DIR}/liblidar_ground.so" DESTINATION lib ${CMAKE_INSTALL_LIBDIR})