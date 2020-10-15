project(robosense)

#include drivers directories
include_directories(${drivers_BINARY_DIR})
include_directories(${drivers_SOURCE_DIR})

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/rs_driver/cmake)
include(rs_driver)
include_directories(${rs_driver_INCLUDE_DIRS})

#build tools
add_subdirectory(tools)

#collect source files
file(GLOB COMMON_SRCS
    "${drivers_SOURCE_DIR}/common/configs/*.cc"
    "${drivers_SOURCE_DIR}/common/time/*.cc"
    "${drivers_SOURCE_DIR}/common/util/*.cc"
	)

#build pandar_sdk library
add_library(robosense_driver_component SHARED onboard/robosense_driver_component.cc ${COMMON_SRCS})
target_link_libraries(robosense_driver_component
    ${rs_driver_LIBRARIES}
    boost_system
    pcap 
    pthread
    drivers_proto
    cyber 
    cyber_proto
    gflags
)

install(FILES "${drivers_BINARY_DIR}/librobosense_driver_component.so" DESTINATION lib ${CMAKE_INSTALL_LIBDIR})