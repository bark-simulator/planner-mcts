cc_library(
    name = "hypothesis_idm",
    hdrs = [
        "hypothesis_idm.hpp"
    ],
    srcs = [
        "hypothesis_idm.cpp",
    ],
    deps = [
        "//bark_mcts/models/behavior/hypothesis:hypothesis",
        "@bark_project//bark/models/behavior/idm/stochastic:stochastic",
    ],
    visibility = ["//visibility:public"],
)

cc_library(
    name="include",
    hdrs=glob(["*.hpp"]),
    visibility = ["//visibility:public"],
)