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

