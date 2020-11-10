project(image_saliency)

#include ai_algorithm directories
include_directories(${ai_algorithm_BINARY_DIR})
include_directories(${ai_algorithm_SOURCE_DIR})

#collect source files
file(GLOB_RECURSE SALIENCY_SRCS ${PROJECT_SOURCE_DIR}/*.cpp)
list(REMOVE_ITEM SALIENCY_SRCS ${PROJECT_SOURCE_DIR}/tools/*.cpp)

#build SALIENCY_SRCS library
add_library(image_saliency SHARED ${SALIENCY_SRCS})
target_link_libraries(image_saliency
    opencv_core
    opencv_imgcodecs 
    opencv_imgproc
)

install(DIRECTORY "${PROJECT_SOURCE_DIR}" DESTINATION include ${CMAKE_INSTALL_INCLUDEDIR} FILES_MATCHING PATTERN "*.h")
install(FILES "${ai_algorithm_BINARY_DIR}/libimage_saliency.so" DESTINATION lib ${CMAKE_INSTALL_LIBDIR})