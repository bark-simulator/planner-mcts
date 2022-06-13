cc_library(
    name = "behavior_action_store",
    srcs = [
        "behavior_action_store.cpp",
    ],
    hdrs = [
        "behavior_action_store.hpp",
    ],
    deps = [
        "@bark_project//bark/models/behavior:behavior",
        "@bark_project//bark/models/dynamic:dynamic",
    ],
    visibility = ["//visibility:public"],
)


cc_library(
    name="include",
    hdrs=glob(["*.hpp"]),
    visibility = ["//visibility:public"],
)