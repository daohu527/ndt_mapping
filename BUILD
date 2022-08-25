load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])


cc_binary(
    name = "ndt_mapping_component",
    srcs = [
      "ndt_mapping_component.cc",
      "ndt_mapping_component.h",
    ],
    linkopts = [
      "-llz4",
    ],
    copts = ["-DMODULE_NAME=\\\"ndt_mapping\\\""],
    deps = [
      "//cyber/common:log",
      "//external:gflags",
      "@pcl",
      "@eigen",
    ],
)
