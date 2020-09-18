project(pandar)

#include drivers directories
include_directories(${drivers_BINARY_DIR})
include_directories(${drivers_SOURCE_DIR})

#build pandar
add_subdirectory(pandar_sdk)

#collect source files
file(GLOB COMMON_SRCS
    "${drivers_SOURCE_DIR}/common/configs/*.cc"
    "${drivers_SOURCE_DIR}/common/time/*.cc"
    "${drivers_SOURCE_DIR}/common/util/*.cc"
	)

#build pandar_sdk library
add_library(pandarsdk_driver_component SHARED onboard/pandarsdk_driver_component.cc ${COMMON_SRCS})
target_link_libraries(pandarsdk_driver_component 
    pandar_sdk
    drivers_proto
    cyber 
    cyber_proto
    gflags
)

install(FILES "${drivers_BINARY_DIR}/libpandarsdk_driver_component.so" DESTINATION lib ${CMAKE_INSTALL_LIBDIR})