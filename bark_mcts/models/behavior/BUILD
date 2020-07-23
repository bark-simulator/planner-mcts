cc_library(
    name = "planner_uct_single_agent",
    srcs = ["behavior_uct_single_agent.cpp"],
    hdrs = ["behavior_uct_single_agent.hpp"],
    copts = ["-Iexternal/bark_project"],
    deps= [
        "@bark_project//bark:include",
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
        "@bark_project//bark:include",
        "@mamcts_project//mcts:mamcts",
        ":planner_uct_single_agent_base",
    ],
    visibility = ["//visibility:public"],
)




cc_library(
    name = "planner_uct_single_agent_base",
    srcs = [
        "behavior_uct_single_agent_base.cpp",
    ],
    hdrs = [
        "behavior_uct_single_agent_base.hpp",
    ],
    copts = ["-Iexternal/bark_project"],
    deps= [
        "//bark_mcts/models/behavior/param_config:param_config",
        "//bark_mcts/models/behavior/mcts_state:mcts_state_single_agent",
        "@bark_project//bark:include",
        "@bark_ml//src/observers:nearest_observer",
        "@bark_project//modules:include",
        "@mamcts_project//mcts:mamcts",
        "@bark_ml//src/model_loader:ModelLoader"
    ],
    copts = ["-Iexternal/bark_project"],
    visibility = ["//visibility:public"],
)