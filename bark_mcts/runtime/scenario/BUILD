py_library(
    name = "config_readers",
    srcs = [
     "behavior_space_sampling.py"
     ],
    data = ["@bark_project//bark:generate_core"],
    imports = [],
    deps = [
        "@bark_project//bark/runtime/scenario:scenario",
        "//bark_mcts/models/behavior/hypothesis/behavior_space:behavior_space"
        ],
    visibility = ["//visibility:public"],
)


py_library(
    name = "scenario_set_generation",
    srcs = [
     "scenario_set_generation.py"
     ],
    data = ["@bark_project//bark:generate_core"],
    imports = [],
    deps = [
        "@bark_project//bark/runtime/scenario:scenario",
        "//bark_mcts/models/behavior/hypothesis/behavior_space:behavior_space"
        ],
    visibility = ["//visibility:public"],
)