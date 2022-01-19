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
  deps = [
    "@local_config_pcl//:pcl",
    "@eigen",
  ]
)

cc_binary(
    name = "ndt_mapping",
    srcs = [
        "ndt_mapping.cc",
    ],
    copts = ["-DMODULE_NAME=\\\"ndt_mapping\\\""],
    deps = [
      ":async_buffer",
      "//cyber/common:log",
      "//modules/localization/msf/common/io:common_io",
      "//external:gflags",
      "@eigen",
    ],
)
