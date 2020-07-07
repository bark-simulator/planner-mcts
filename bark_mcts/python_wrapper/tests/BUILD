package(default_visibility = ["//visibility:public"])



# the following rules are only for testing purposes
cc_binary(
  name = "planner_uct.so",
  srcs = ["python_test_module.cpp"],
  deps = [
    "//bark_mcts/python_wrapper:planner_uct",
    "//bark_mcts/models/behavior/hypothesis/idm:hypothesis_idm",
    "//bark_mcts/models/behavior/action_store:behavior_action_store",
    "@bark_project//bark/python_wrapper:behavior",
    "@bark_project//bark/models/behavior/constant_velocity:constant_velocity",
    "@bark_project//bark/models/behavior/motion_primitives:motion_primitives",
    "@bark_project//bark/models/behavior/dynamic_model:dynamic_model",
    "@bark_project//bark/models/behavior/idm:idm_classic",
    "@bark_project//bark/models/behavior/idm:idm_lane_tracking",
    "@bark_project//bark/models/behavior/rule_based:lane_change_behavior",
    "@bark_project//bark/models/behavior/rule_based:intersection_behavior",
    "@bark_project//bark/models/behavior/rule_based:mobil_behavior",
    "@bark_project//bark/models/behavior/static_trajectory",
    "@bark_project//bark/models/behavior/idm/stochastic:stochastic",
    "@bark_project//bark/world:world",
    "@bark_project//bark/world/evaluation:evaluator_goal_reached",
    "@bark_project//bark/world/evaluation:evaluator_collision_ego_agent",
    "@bark_project//bark/world/evaluation:evaluator_drivable_area",
  ],
  linkshared = 1,
  linkstatic = 1
)


py_test(
  name = "pickle_unpickle_test",
  srcs = ["pickle_unpickle_test.py"],
  data = ['@bark_project//bark:generate_core'],
  imports = ['../python/'],
  deps=[
    "@bark_project//bark/runtime:runtime",
  ]
)
