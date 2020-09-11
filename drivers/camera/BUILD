load("//tools:cpplint.bzl", "cpplint")
load("//tools:cuda_library.bzl", "cuda_library")

package(default_visibility = ["//visibility:public"])

cuda_library(
     name = "cuda_yuyv2rgb_lib",
     srcs = [
         "image_gpu.cu",
     ],
     hdrs = [
         "image_gpu.h",
     ],
     deps = [
         "//cyber",
         "@cuda",
     ],
 )

cc_library(
    name = "yuyv2rgb_lib",
    srcs = [
        ":cuda_yuyv2rgb_lib",
    ],
    hdrs = [
         "image_gpu.h",
    ],
    linkstatic = 1,
)

cc_binary(
    name = "libcamera_component.so",
    linkopts = [
        "-lswscale",
        "-lavformat",
        "-lavcodec",
        "-latomic",
    ],
    linkshared = True,
    linkstatic = False,
    deps = [
        ":camera_component_lib",
        ":compress_component_lib",
    ],
)

cc_library(
    name = "camera_component_lib",
    srcs = ["camera_component.cc"],
    hdrs = ["camera_component.h"],
    copts = ['-DMODULE_NAME=\\"camera\\"'],
    deps = [
        ":camera",
        "//cyber",
        "//modules/drivers/proto:sensor_proto",
    ],
)

cc_library(
    name = "compress_component_lib",
    srcs = ["compress_component.cc"],
    hdrs = ["compress_component.h"],
    copts = ['-DMODULE_NAME=\\"camera\\"'],
    deps = [
        "//cyber",
        "//modules/drivers/camera/proto:camera_proto",
        "//modules/drivers/proto:sensor_proto",
        "@opencv2//:core",
        "@opencv2//:highgui",
    ],
)

cc_library(
    name = "camera",
    srcs = [
        "usb_cam.cc",
        "util.cc",
        "get_gateway_ts.cc",
        "image_gpu.h",
        "time_ts_service.cc"
    ],
    hdrs = [
        "usb_cam.h",
        "util.h",
        "get_gateway_ts.h",
        "time_ts_service.h"
    ],
    copts = ["-mavx2"],
    deps = [
        "//cyber",
        "//modules/drivers/camera/proto:camera_proto",
        ":yuyv2rgb_lib",
        "@adv_plat",
    ],
)

cpplint()
