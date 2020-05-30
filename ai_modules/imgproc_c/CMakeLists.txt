project(imgproc_c)

#include ai_algorithm directories
include_directories(${ai_algorithm_BINARY_DIR})
include_directories(${ai_algorithm_SOURCE_DIR})

#collect source files
file(GLOB IMGPROC_SRCS 
    "${PROJECT_SOURCE_DIR}/common/*.c"
	"${PROJECT_SOURCE_DIR}/base/*.c"
	"${PROJECT_SOURCE_DIR}/cvtcolor/*.c"
	"${PROJECT_SOURCE_DIR}/image_resize/*.c"
	"${PROJECT_SOURCE_DIR}/filtering/*.c"
	"${PROJECT_SOURCE_DIR}/features2d/*.c"
	"${PROJECT_SOURCE_DIR}/optical_flow/*.c"
	"${PROJECT_SOURCE_DIR}/clustering/*.c"
	"${PROJECT_SOURCE_DIR}/tracking/*.c"
	)

#build cyber library
add_library(imgproc_c SHARED ${IMGPROC_SRCS})
target_link_libraries(imgproc_c)

install(DIRECTORY "${ai_algorithm_SOURCE_DIR}/imgproc_c" DESTINATION include ${CMAKE_INSTALL_INCLUDEDIR} FILES_MATCHING PATTERN "*.h")
install(FILES "${ai_algorithm_BINARY_DIR}/libimgproc_c.so" DESTINATION lib ${CMAKE_INSTALL_LIBDIR})