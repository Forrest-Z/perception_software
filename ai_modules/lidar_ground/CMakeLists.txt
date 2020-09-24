project(lidar_ground)

#include ai_algorithm directories
include_directories(${ai_algorithm_BINARY_DIR})
include_directories(${ai_algorithm_SOURCE_DIR})

add_subdirectory(ground)

#collect tools files
file(GLOB TOOLS_SRCS
    "${ai_algorithm_SOURCE_DIR}/lidar_ground/tools/pcl_viewer.cc"
)

add_executable(lidar_ground_segement tools/lidar_ground_segement.cc ${TOOLS_SRCS})
target_link_libraries(lidar_ground_segement
    lidar_common
    lidar_ground
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
