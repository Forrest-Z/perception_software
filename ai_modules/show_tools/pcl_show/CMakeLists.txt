project(lidar_tools)

#include ai_algorithm directories
include_directories(${ai_algorithm_BINARY_DIR})
include_directories(${ai_algorithm_SOURCE_DIR})

#collect source files
file(GLOB COMMON_SRCS
    "${ai_algorithm_SOURCE_DIR}/base/*.cc"
    "${ai_algorithm_SOURCE_DIR}/common/*.cc"
)

#collect source files
file(GLOB TOOLS_SRCS 
  "${PROJECT_SOURCE_DIR}/*.cc"
)

add_executable(show_obstacle_perception ${TOOLS_SRCS} ${COMMON_SRCS})
target_link_libraries(show_obstacle_perception 
    ai_proto
    cyber 
    cyber_proto
    glog
    gflags
    opencv_core
    opencv_imgcodecs 
    opencv_imgproc
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
    pcl_visualization
    Qt5::Sql
)
