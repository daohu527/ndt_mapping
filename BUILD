load("//tools:cpplint.bzl", "cpplint")

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
    "@pcl",
    "@eigen",
  ]
)

cc_binary(
    name = "ndt_mapping",
    srcs = [
      "ndt_mapping.cc",
    ],
    linkopts = [
      "-L/usr/local/apollo/boost/lib",
      "-llz4",
    ],
    copts = ["-DMODULE_NAME=\\\"ndt_mapping\\\""],
    deps = [
      ":async_buffer",
      "//cyber/common:log",
      "//external:gflags",
      "@pcl",
      "@eigen",
    ],
)
