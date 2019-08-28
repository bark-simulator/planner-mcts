cc_library(
    name = "planner_uct_single_agent",
    srcs = [
        "behavior_uct_single_agent.cpp",
        "single_agent_mcts_state.cpp"
    ],
    hdrs = [
        "single_agent_mcts_state.hpp",
        "behavior_uct_single_agent.hpp",
    ],
    copts = ["-Iexternal/bark_project"],
    deps= [
        "@bark_project//modules/models/behavior:behavior",
        "@bark_project//modules/models/behavior/motion_primitives:motion_primitives",
        "@bark_project//modules/models/behavior/constant_velocity:constant_velocity",
        "@bark_project//modules/world:world",
        "@bark_project//modules/world/evaluation:evaluator_goal_reached",
        "@bark_project//modules/world/evaluation:evaluator_collision_ego_agent",
        "@bark_project//modules/world/evaluation:evaluator_collision_driving_corridor",
        "@mamcts_project//mcts:mcts",
    ],
    visibility = ["//visibility:public"],
)