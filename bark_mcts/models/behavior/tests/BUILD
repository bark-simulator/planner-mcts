#cc_test(
#    name = "single_agent_test",
#    srcs = [
#        "single_agent_test.cc",
#    ],
#    copts = ["-Iexternal/gtest/include", "-O3"],
#    deps = [
#        "//bark_mcts/models/behavior:planner_uct_single_agent",
#        "@bark_project//bark/world:world",
#        "@bark_project//bark/world/tests:make_test_world",
#        "@bark_project//bark/models/behavior:behavior",
#        "@bark_project//bark/models/behavior/motion_primitives:motion_primitives",
#        "@bark_project//bark/models/behavior/constant_velocity:constant_velocity",
#        "@bark_project//bark/world/evaluation:evaluator_goal_reached",
#        "@bark_project//bark/models/behavior/idm:idm_lane_tracking",
#        "@bark_project//bark/world/evaluation:evaluator_collision_ego_agent",
#        "@bark_project//bark/world/evaluation:evaluator_drivable_area",
#        "@bark_project//bark/world/goal_definition:goal_definition",
#        "@gtest//:gtest_main",
#    ],
#)

#cc_test(
#    name = "single_agent_macro_actions_test",
#    srcs = [
#        "single_agent_macro_actions_test.cc"
#    ],
#    copts = ["-Iexternal/gtest/include"],
#    deps = [
#        "//bark_mcts/models/behavior:planner_uct_single_agent_macro_actions",
#        "@bark_project//bark/world:world",
#        "@bark_project//bark/world/tests:make_test_world",
#        "@bark_project//bark/models/behavior:behavior",
#        "@bark_project//bark/models/behavior/motion_primitives:motion_primitives",
#        "@bark_project//bark/models/behavior/idm:idm_lane_tracking",
#        "@bark_project//bark/models/behavior/constant_velocity:constant_velocity",
#        "@bark_project//bark/world/evaluation:evaluator_goal_reached",
#        "@bark_project//bark/world/evaluation:evaluator_collision_ego_agent",
#        "@bark_project//bark/world/evaluation:evaluator_drivable_area",
#        "@bark_project//bark/world/goal_definition:goal_definition",
#        "@gtest//:gtest_main",
#    ],
#)

cc_library(
    name="test_helpers",
    srcs=[
        "test_helpers.cpp"
    ],
    hdrs = [
        "test_helpers.hpp"
    ],
    deps = [
        "//bark_mcts/models/behavior/hypothesis/idm:hypothesis_idm",
        "//bark_mcts/models/behavior/action_store:behavior_action_store",
        "@bark_project//bark/geometry",
        "@bark_project//bark/models/behavior/constant_velocity:constant_velocity",
        "@bark_project//bark/models/behavior/motion_primitives:motion_primitives",
        "@bark_project//bark/models/behavior/dynamic_model:dynamic_model",
        "@bark_project//bark/models/behavior/idm:idm_classic",
        "@bark_project//bark/models/behavior/idm:idm_lane_tracking",
        "@bark_project//bark/models/behavior/rule_based:lane_change_behavior",
        "@bark_project//bark/models/behavior/rule_based:intersection_behavior",
        "@bark_project//bark/models/behavior/rule_based:mobil_behavior",
        "@bark_project//bark/models/behavior/static_trajectory",
        "@bark_project//bark/models/execution/interpolation:interpolation",
        "@bark_project//bark/models/behavior/idm/stochastic:stochastic",
        "@bark_project//bark/world/tests:make_test_xodr_map",
        "@com_github_eigen_eigen//:eigen",
    ],
    visibility = ["//visibility:public"],
)


cc_test(
    name = "behavior_hypothesis_test",
    srcs = [
        "behavior_hypothesis_test.cc",
    ],
    copts = ["-Iexternal/gtest/include"],
    deps = [
        "//bark_mcts/models/behavior/hypothesis/idm:hypothesis_idm",
        "@bark_project//bark/geometry",
        "@bark_project//bark/models/behavior/constant_velocity:constant_velocity",
        "@bark_project//bark/models/execution/interpolation:interpolation",
        "@gtest//:gtest_main",
        "@com_github_eigen_eigen//:eigen",
        "@bark_project//bark/world/tests:make_test_world",
    ],
)


py_test(
  name = "py_behavior_space_test",
  srcs = ["py_behavior_space_test.py"],
  data = ['@bark_project//bark:generate_core',
          '//bark_mcts/models/behavior/hypothesis/behavior_space:behavior_space'],
  imports = ['../../../python/'],
  deps = [
      "@bark_project//bark/runtime/commons:commons",
      "//util/debug:debug_settings",
    ],
)

py_test(
  name = "py_scenario_risk_function_test",
  srcs = ["py_scenario_risk_function_test.py"],
  data = ['@bark_project//bark:generate_core',
          '//bark_mcts/models/behavior/hypothesis/behavior_space:behavior_space'
        ],
  imports = ['../../../python/'],
  deps = [
      "@bark_project//bark/runtime/commons:commons",
      "//util/debug:debug_settings",
    ],
)

cc_test(
    name = "behavior_uct_hypothesis_test",
    srcs = [
        "behavior_uct_hypothesis_test.cc"
    ],
    copts = ["-Iexternal/gtest/include"],
    deps = [
        "//bark_mcts/models/behavior:planner_uct_hypothesis",
        "//bark_mcts/models/behavior/action_store:behavior_action_store",
        "//bark_mcts/models/behavior/hypothesis/idm:hypothesis_idm",
        ":test_helpers",
        "@bark_project//bark/world:world",
        "@bark_project//bark/world/tests:make_test_world",
        "@bark_project//bark/models/behavior:behavior",
        "@bark_project//bark/models/behavior/motion_primitives:motion_primitives",
        "@bark_project//bark/models/behavior/idm/stochastic:stochastic",
        "@bark_project//bark/models/behavior/idm:idm_lane_tracking",
        "@bark_project//bark/models/execution/interpolation:interpolation",
        "@bark_project//bark/world/evaluation:evaluator_goal_reached",
        "@bark_project//bark/world/evaluation:evaluator_collision_ego_agent",
        "@bark_project//bark/world/evaluation:evaluator_drivable_area",
        "@bark_project//bark/world/goal_definition:goal_definition",
        "@gtest//:gtest_main",
    ],
)