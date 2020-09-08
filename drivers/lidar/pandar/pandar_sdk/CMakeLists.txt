project(pandar_sdk)

#include drivers directories
include_directories(${drivers_BINARY_DIR})
include_directories(${drivers_SOURCE_DIR})

#collect source files
file(GLOB PANDAR_SDK_SRCS 
    "${PROJECT_SOURCE_DIR}/*.cc"
	"${PROJECT_SOURCE_DIR}/*.c"
)

#build pandar_sdk library
add_library(${PROJECT_NAME} SHARED ${PANDAR_SDK_SRCS})
target_link_libraries(${PROJECT_NAME} 
	${Boost_LIBRARIES}
	${PCL_IO_LIBRARIES}
)

install(DIRECTORY "${drivers_SOURCE_DIR}/lidar/pandar/pandar_sdk" DESTINATION include ${CMAKE_INSTALL_INCLUDEDIR} FILES_MATCHING PATTERN "*.h")
install(FILES "${drivers_BINARY_DIR}/libpandar_sdk.so" DESTINATION lib ${CMAKE_INSTALL_LIBDIR})