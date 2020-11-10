project(lidar_tracking)

#include ai_algorithm directories
include_directories(${ai_algorithm_BINARY_DIR})
include_directories(${ai_algorithm_SOURCE_DIR})

add_subdirectory(kalman_tracking)

#collect tools files
file(GLOB TOOLS_SRCS
    "${ai_algorithm_SOURCE_DIR}/show_tools/pcl_show/pcl_show.cc"
)

add_executable(lidar_tracking_objects tools/lidar_tracking_objects.cc ${TOOLS_SRCS})
target_link_libraries(lidar_tracking_objects
    ai_proto
    lidar_common
    lidar_ground
    lidar_clustering
    lidar_kalman_tracking
    vtkCommonCore
    vtkCommonMath
    vtkCommonSystem
    vtkFiltersCore
    vtkFiltersGeneral
    vtkglew
    vtkRenderingOpenGL2
    vtkRenderingContextOpenGL2
    vtkInteractionStyle
    boost_system
    pcl_common
    pcl_io
    pcl_filters
    pcl_sample_consensus
    pcl_segmentation
    pcl_visualization
    Qt5::Sql
)
