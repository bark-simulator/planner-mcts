cc_library(
    name = "planner_uct_single_agent",
    srcs = [
        "behavior_uct_single_agent.cpp",
        "mcts_state_single_agent.cpp"
    ],
    hdrs = [
        "mcts_state_single_agent.hpp",
        "behavior_uct_single_agent.hpp",
        "mcts_parameters_from_param_server.hpp"
    ],
    copts = ["-Iexternal/bark_project"],
    deps= [
        "@bark_project//modules:include",
        "@mamcts_project//mcts:mamcts",
    ],
    visibility = ["//visibility:public"],
)