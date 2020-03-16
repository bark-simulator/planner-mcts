cc_test(
    name = "single_agent_test",
    srcs = [
        "single_agent_test.cc",
    ],
    copts = ["-Iexternal/gtest/include", "-O3"],
    deps = [
        "//src:planner_uct_single_agent",
        "@bark_project//modules/world:world",
        "@bark_project//modules/world/tests:make_test_world",
        "@bark_project//modules/models/behavior:behavior",
        "@bark_project//modules/models/behavior/motion_primitives:motion_primitives",
        "@bark_project//modules/models/behavior/mobil:mobil",
        "@bark_project//modules/models/behavior/constant_velocity:constant_velocity",
        "@bark_project//modules/world/evaluation:evaluator_goal_reached",
        "@bark_project//modules/world/evaluation:evaluator_collision_ego_agent",
        "@bark_project//modules/world/evaluation:evaluator_drivable_area",
        "@bark_project//modules/world/goal_definition:goal_definition",
        "@gtest//:main",
    ],
)

cc_test(
    name = "single_agent_macro_actions_test",
    srcs = [
        "single_agent_macro_actions_test.cc"
    ],
    copts = ["-Iexternal/gtest/include"],
    deps = [
        "//src:planner_uct_single_agent_macro_actions",
        "@bark_project//modules/world:world",
        "@bark_project//modules/world/tests:make_test_world",
        "@bark_project//modules/models/behavior:behavior",
        "@bark_project//modules/models/behavior/motion_primitives:motion_primitives",
        "@bark_project//modules/models/behavior/mobil:mobil",
        "@bark_project//modules/models/behavior/constant_velocity:constant_velocity",
        "@bark_project//modules/world/evaluation:evaluator_goal_reached",
        "@bark_project//modules/world/evaluation:evaluator_collision_ego_agent",
        "@bark_project//modules/world/evaluation:evaluator_drivable_area",
        "@bark_project//modules/world/goal_definition:goal_definition",
        "@gtest//:main",
    ],
)

cc_test(
    name = "hypothesis_agent_test",
    srcs = [
        "hypothesis_agent_test.cc"
    ],
    copts = ["-Iexternal/gtest/include"],
    deps = [
        "//src:planner_uct_hypothesis",
        "@bark_project//modules/world:world",
        "@bark_project//modules/world/tests:make_test_world",
        "@bark_project//modules/models/behavior:behavior",
        "@bark_project//modules/models/behavior/motion_primitives:motion_primitives",
        "@bark_project//modules/models/behavior/mobil:mobil",
        "@bark_project//modules/models/behavior/action_store:behavior_action_store",
        "@bark_project//modules/models/behavior/hypothesis/idm:hypothesis_idm",
        "@bark_project//modules/models/behavior/constant_velocity:constant_velocity",
        "@bark_project//modules/models/behavior/idm:idm_lane_tracking",
        "@bark_project//modules/world/evaluation:evaluator_goal_reached",
        "@bark_project//modules/world/evaluation:evaluator_collision_ego_agent",
        "@bark_project//modules/world/evaluation:evaluator_drivable_area",
        "@bark_project//modules/world/goal_definition:goal_definition",
        "@gtest//:main",
    ],
)