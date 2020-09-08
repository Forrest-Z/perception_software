
set(OpenCV_DIR /home/lpj/Software/opencv34/share/OpenCV)
find_package(OpenCV REQUIRED CONFIG)
message(STATUS "Found OpenCV ${OpenCV_VERSION}")
message(STATUS "${OpenCV_INCLUDE_DIRS}, ${OpenCV_LIBRARIES}")
include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_LIBRARIES})

find_package(Boost REQUIRED)
message(STATUS "Found Boost ${Boost_VERSION}")
message(STATUS "${Boost_INCLUDE_DIRS}, ${Boost_LIBRARIES}")
include_directories(${Boost_INCLUDE_DIRS})
link_directories(${Boost_LIBRARIES})

set(PCL_DIR /home/lpj/Software/pcl_1_9/share/pcl-1.9)
find_package(PCL REQUIRED COMPONENTS common io CONFIG)
message(STATUS "Found PCL ${PCL_VERSION}")
message(STATUS "${PCL_INCLUDE_DIRS}, ${PCL_LIBRARIES}")
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_IO_LIBRARIES})

set(yaml-cpp_DIR /home/lpj/Software/yaml_cpp/lib/cmake/yaml-cpp)
find_package(yaml-cpp REQUIRED CONFIG)
message(STATUS "Found yaml-cpp ${YAML_CPP_VERSION}")
message(STATUS "${YAML_CPP_INCLUDE_DIR}, ${YAML_CPP_LIBRARIES}")
include_directories(${YAML_CPP_INCLUDE_DIR})
link_directories(${YAML_CPP_LIBRARIES})