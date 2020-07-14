package(default_visibility = ["//visibility:public"])

py_test(
  name = "py_scenario_generation_tests",
  srcs = ["py_scenario_generation_tests.py"],
  data = ['@bark_project//bark:generate_core',
         '@bark_project//bark/runtime/tests:xodr_data'],
  deps = ["@bark_project//bark/runtime/commons:commons",
          "@bark_project//bark/runtime/scenario/scenario_generation:scenario_generation",
          "//bark_mcts/runtime/scenario:config_readers"],
)

