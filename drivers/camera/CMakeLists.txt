project(camera_component)

#include drivers directories
include_directories(${drivers_BINARY_DIR})
include_directories(${drivers_SOURCE_DIR})

add_subdirectory(yuv2rgb)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O2 -fopenmp -mavx2 -fPIC")

find_package(PkgConfig REQUIRED)
pkg_check_modules(avcodec libavcodec REQUIRED)
pkg_check_modules(swscale libswscale REQUIRED)
include_directories(${avcodec_INCLUDE_DIRS})
include_directories(${swscale_INCLUDE_DIRS})

include_directories(/home/lpj/Software/adv_plat)
link_directories(/home/lpj/Software/adv_plat/lib)

find_package(CUDA REQUIRED)
message("CUDA_INCLUDE_DIRS : " ${CUDA_INCLUDE_DIRS})
include_directories(${CUDA_INCLUDE_DIRS})

#include(cmake/FindV4L2.cmake)
#find_package(V4L2 REQUIRED)
#message("V4L2_FOUND : " ${V4L2_FOUND})
#message("V4L2_INCLUDE_DIR : " ${V4L2_INCLUDE_DIR})
#message("V4L2_LIBRARIES : " ${V4L2_LIBRARIES})
#message("V4L2_LIBRARY : " ${V4L2_LIBRARY})
#include_directories(${V4L2_INCLUDE_DIR})

#collect source files
file(GLOB CAMERA_SRCS 
    "${PROJECT_SOURCE_DIR}/*.cc"
)

#build camera_component library
add_library(camera_component SHARED ${CAMERA_SRCS})
target_link_libraries(camera_component 
    yuv2rgb
    drivers_proto
    cyber 
    cyber_proto
    adv_bcan
    adv_trigger
    ${avcodec_LIBRARIES}
    ${swscale_LIBRARIES}
    opencv_core 
    opencv_imgcodecs 
    opencv_imgproc
)

install(FILES "${drivers_BINARY_DIR}/libcamera_component.so" DESTINATION lib ${CMAKE_INSTALL_LIBDIR})