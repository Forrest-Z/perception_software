project(lidar_kalman_tracking)

#include ai_algorithm directories
include_directories(${ai_algorithm_BINARY_DIR})
include_directories(${ai_algorithm_SOURCE_DIR})

#collect source files
file(GLOB LIDAR_KALMAN_TRACKING_SRCS 
    "${PROJECT_SOURCE_DIR}/*.cc"
	)

#build lidar_kalman_tracking library
add_library(lidar_kalman_tracking SHARED ${LIDAR_KALMAN_TRACKING_SRCS})
target_link_libraries(lidar_kalman_tracking
    lidar_common
    opencv_core
    opencv_imgcodecs 
    opencv_imgproc
    opencv_video
)

install(DIRECTORY "${PROJECT_SOURCE_DIR}" DESTINATION include ${CMAKE_INSTALL_INCLUDEDIR} FILES_MATCHING PATTERN "*.h")
install(FILES "${ai_algorithm_BINARY_DIR}/liblidar_kalman_tracking.so" DESTINATION lib ${CMAKE_INSTALL_LIBDIR})