load("@rules_cc//cc:defs.bzl", "cc_library")

package(default_visibility = ["//visibility:public"])

licenses(["notice"])

cc_library(
    name = "pcl",
    includes = [
        ".",
    ],
    linkopts = [
        "-L/usr/local/lib",
        "-lboost_system",
        "-lpcl_common",
        "-lpcl_features",
        "-lpcl_filters",
        "-lpcl_io_ply",
        "-lpcl_io",
        "-lpcl_kdtree",
        "-lpcl_keypoints",
        "-lpcl_octree",
        "-lpcl_outofcore",
        "-lpcl_people",
        "-lpcl_recognition",
        "-lpcl_registration",
        "-lpcl_sample_consensus",
        "-lpcl_search",
        "-lpcl_segmentation",
        "-lpcl_surface",
        "-lpcl_tracking",
        "-lpcl_visualization",
    ],
)
