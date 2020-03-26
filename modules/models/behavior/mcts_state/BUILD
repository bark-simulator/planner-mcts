cc_library(
    name = "mcts_state_hypothesis",
    srcs = ["behavior_uct_single_agent.cpp"],
    hdrs = ["behavior_uct_single_agent.hpp"],
    copts = ["-Iexternal/bark_project"],
    deps= [
        "@bark_project//modules:include",
        "@mamcts_project//mcts:mamcts",
        ":planner_uct_single_agent_base",
    ],
    visibility = ["//visibility:public"],
)

