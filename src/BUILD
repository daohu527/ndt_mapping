load("@rules_cc//cc:defs.bzl", "cc_library", "cc_binary")

package(default_visibility = ["//visibility:public"])

cc_library(
  name = "async_buffer",
  srcs = [
    "async_buffer.cc",
  ],
  hdrs = [
    "async_buffer.h",
  ],
  linkopts = [
    "-lpthread",
  ],
  deps = [
    "//third_party/pcl",
  ]
)

cc_binary(
    name = "ndt_mapping",
    srcs = [
      "ndt_mapping.cc",
    ],
    linkopts = [
    ],
    copts = ["-DMODULE_NAME=\\\"ndt_mapping\\\""],
    deps = [
      ":async_buffer",
      "//third_party/gflags",
      "//third_party/pcl",
      "//third_party/eigen",
    ],
)
