cc_library(
    name = "planner_uct_single_agent",
    srcs = [
        "behavior_uct_single_agent.cpp"
    ],
    hdrs = [
        "bark_mamcts_state.hpp",
        "behavior_uct_single_agent.hpp",
    ],
    copts = ["-Iexternal/bark_project"],
    deps= [
        "@bark_project//modules/models/behavior:behavior",
        "@mamcts_project//mcts:mcts",
    ],
    visibility = ["//visibility:public"],
)