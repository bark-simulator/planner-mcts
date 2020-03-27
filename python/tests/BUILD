package(default_visibility = ["//visibility:public"])



# the following rules are only for testing purposes
cc_binary(
  name = "planner_uct.so",
  srcs = ["python_test_module.cpp"],
  deps = [
    "//python:planner_uct",
    "@bark_project//python:behavior",
    "@bark_project//modules/models/behavior:behavior",
    "@bark_project//modules/models/behavior/motion_primitives:motion_primitives",
    "@bark_project//modules/models/behavior/constant_velocity:constant_velocity",
    "@bark_project//modules/models/behavior/mobil:mobil",
    "@bark_project//modules/world:world",
    "@bark_project//modules/world/evaluation:evaluator_goal_reached",
    "@bark_project//modules/world/evaluation:evaluator_collision_ego_agent",
    "@bark_project//modules/world/evaluation:evaluator_drivable_area",
  ],
  linkshared = 1,
  linkstatic = 1
)

py_test(
  name = "python_import_test",
  srcs = ["python_import_test.py"],
  data = ['//python/tests:planner_uct.so'],
  imports = ['../python/'],
)

py_test(
  name = "pickle_unpickle_test",
  srcs = ["pickle_unpickle_test.py"],
  data = ['@bark_project//python:bark.so'],
  imports = ['../python/'],
  deps=[
    "@bark_project//modules/runtime:runtime",
  ]
)
