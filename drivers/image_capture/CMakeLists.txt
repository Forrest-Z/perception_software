project(image_capture)

#include drivers directories
include_directories(${drivers_BINARY_DIR})
include_directories(${drivers_SOURCE_DIR})

#set( CMAKE_BUILD_TYPE "Debug" )

add_executable(image_capture ${PROJECT_SOURCE_DIR}/image_capture.cc)
target_link_libraries(image_capture 
    opencv_core 
    opencv_imgcodecs 
    opencv_imgproc
    opencv_highgui
    opencv_videoio)





