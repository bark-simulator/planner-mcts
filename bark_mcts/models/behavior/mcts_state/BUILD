cc_library(
    name = "mcts_state_hypothesis",
    hdrs = [
            "mcts_state_hypothesis.hpp",
            "mcts_state_hypothesis.cpp",
            ],
    copts = ["-Iexternal/bark_project"],
    deps= [
        ":mcts_state_base",
        "//bark_mcts/models/behavior/hypothesis:hypothesis",
        "//bark_mcts/models/behavior/action_store:behavior_action_store",
        "@bark_project//bark:include",
        "@mamcts_project//mcts:mamcts",
    ],
    visibility = ["//visibility:public"],
)

cc_library(
    name = "mcts_state_risk_constraint",
    srcs = [
            "mcts_state_risk_constraint.cpp",
    ],
    hdrs = [
            "mcts_state_risk_constraint.hpp",
            ],
    copts = ["-Iexternal/bark_project"],
    deps= [
        ":mcts_state_hypothesis",
        "//bark_mcts/models/behavior/hypothesis:hypothesis",
        "//bark_mcts/models/behavior/action_store:behavior_action_store",
        "@bark_project//bark:include",
        "@mamcts_project//mcts:mamcts",
    ],
    visibility = ["//visibility:public"],
)

cc_library(
    name = "mcts_state_base",
    hdrs = [
            "mcts_state_base.hpp",
            ],
    copts = ["-Iexternal/bark_project"],
    deps= [
        "@bark_project//bark:include",
        "@bark_project//bark/world/evaluation/safe_distances:safe_distances",
        "@mamcts_project//mcts:mamcts",
    ],
    visibility = ["//visibility:public"],
)


cc_library(
    name = "mcts_state_cooperative",
    srcs = [
            "mcts_state_cooperative.cpp",
    ],
    hdrs = [
            "mcts_state_cooperative.hpp",
            ],
    copts = ["-Iexternal/bark_project"],
    deps= [
        ":mcts_state_base",
        "@bark_project//bark:include",
        "@mamcts_project//mcts:mamcts",
    ],
    visibility = ["//visibility:public"],
)


cc_library(
    name = "mcts_state_single_agent",
    srcs = ["mcts_state_single_agent.cpp"],
    hdrs = ["mcts_state_single_agent.hpp"],
    copts = ["-Iexternal/bark_project"],
    deps= [
        "@bark_project//bark:include",
        "@mamcts_project//mcts:mamcts",
    ],
    visibility = ["//visibility:public"],
)

