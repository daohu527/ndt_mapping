load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "ndt_mapping",
    srcs = [
        "ndt_mapping.cc",
    ],
    copts = ["-DMODULE_NAME=\\\"ndt_mapping\\\""],    
    deps = [
      "//cyber/common:log",
      "//modules/localization/msf/common/io:localization_msf_common_io",
      "//external:gflags",
      "@eigen",      
      "@pcl",
    ],
)
