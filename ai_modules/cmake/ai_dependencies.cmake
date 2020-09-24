set(OpenCV_DIR /home/lpj/Software/opencv34/share/OpenCV)
find_package(OpenCV REQUIRED CONFIG)
message(STATUS "Found OpenCV ${OpenCV_VERSION}")
message(STATUS "${OpenCV_INCLUDE_DIRS}, ${OpenCV_LIBRARIES}")
include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_LIBRARIES})

set(PCL_DIR /home/lpj/Software/pcl_1_9/share/pcl-1.9)
find_package(PCL REQUIRED COMPONENTS 
             common
             io 
             kdtree 
             filters 
             sample_consensus 
             segmentation 
             keypoints 
             visualization CONFIG)
message(STATUS "Found PCL ${PCL_VERSION}")
message(STATUS "${PCL_INCLUDE_DIRS}, ${PCL_LIBRARIES}")
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARIES})
# Fix a compilation bug under ubuntu 16.04 (Xenial)
list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")

set(Qt5_DIR /opt/Qt5.12.0/5.12.0/gcc_64/lib/cmake/Qt5)
find_package(Qt5 REQUIRED COMPONENTS Core Widgets Gui OpenGL Sql)