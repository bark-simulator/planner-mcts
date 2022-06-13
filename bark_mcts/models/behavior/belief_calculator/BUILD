cc_library(
    name = "belief_calculator",
    srcs = [
        "belief_calculator.cpp",
    ],
    hdrs = [
            "belief_calculator.hpp",
    ],
    copts = ["-Iexternal/bark_project"],
    deps= [
        "//bark_mcts/models/behavior/mcts_state:mcts_state_base",
        "//bark_mcts/models/behavior/param_config:param_config",
        "//bark_mcts/models/behavior/mcts_state:mcts_state_hypothesis",
        "@bark_project//bark:include",
        "@mamcts_project//mcts:mamcts"
    ],
    visibility = ["//visibility:public"],
)