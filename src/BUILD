cc_library(
    name = "planner_uct_single_agent",
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

cc_library(
    name = "planner_uct_single_agent_macro_actions",
    srcs = ["behavior_uct_single_agent_macro_actions.cpp"],
    hdrs = ["behavior_uct_single_agent_macro_actions.hpp"],
    copts = ["-Iexternal/bark_project"],
    deps= [
        "@bark_project//modules:include",
        "@mamcts_project//mcts:mamcts",
        ":planner_uct_single_agent_base",
    ],
    visibility = ["//visibility:public"],
)

cc_library(
    name = "planner_uct_hypothesis",
    srcs = [
        "behavior_uct_hypothesis.cpp",
        "mcts_state_hypothesis.cpp"
    ],
    hdrs = [
            "behavior_uct_hypothesis.hpp",
            "mcts_state_hypothesis.hpp",
            "mcts_parameters_from_param_server.hpp"
    ],
    copts = ["-Iexternal/bark_project"],
    deps= [
        "@bark_project//modules:include",
        "@mamcts_project//mcts:mamcts",
        ":planner_uct_single_agent_base",
    ],
    visibility = ["//visibility:public"],
)




cc_library(
    name = "planner_uct_single_agent_base",
    srcs = [
        "behavior_uct_single_agent_base.cpp",
        "mcts_state_single_agent.cpp"
    ],
    hdrs = [
        "mcts_state_single_agent.hpp",
        "behavior_uct_single_agent_base.hpp",
        "mcts_parameters_from_param_server.hpp"
    ],
    copts = ["-Iexternal/bark_project"],
    deps= [
        "@bark_project//modules:include",
        "@mamcts_project//mcts:mamcts",
    ],
    visibility = ["//visibility:public"],
)