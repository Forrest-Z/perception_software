load("//tools:cpplint.bzl", "cpplint")
load("//tools:qt.bzl", "qt_cc_library")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "show_camera_radar_fusion",
    includes = [
        ".",
    ],
    deps = [
        ":my_show_libs",
    ],
)

qt_cc_library(
    name = "my_show_libs",
    src = glob(
        ["*.cc"],
    ),
    copts = [
        "-Imodules/drivers/proto",
    ],
    hdr = glob([
        "*.h",
    ]),
    includes = [
        ".",
    ],
    linkstatic = 1,
    res = glob([
        "*.qrc",
    ]),
    uis = glob([
        "uis/*.ui",
    ]),
    deps = [
        "//cyber",
        "//modules/drivers/proto:sensor_proto",
        "//modules/map/proto:map_proto",
        "//modules/perception/base",
        "//modules/perception/proto:perception_proto",
        "@eigen",
        "@qt//:qt_gui",
    ],
)


cpplint()
