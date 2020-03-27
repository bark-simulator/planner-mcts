cc_library(
    name = "mcts_state_hypothesis",
    srcs = ["mcts_state_hypothesis.cpp"],
    hdrs = ["mcts_state_hypothesis.hpp"],
    copts = ["-Iexternal/bark_project"],
    deps= [
        "@bark_project//modules:include",
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
        "@bark_project//modules:include",
        "@mamcts_project//mcts:mamcts",
    ],
    visibility = ["//visibility:public"],
)

