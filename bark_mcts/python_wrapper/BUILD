package(default_visibility = ["//visibility:public"])

cc_library(
  name = "planner_uct",
  srcs = [
      "python_planner_uct.cpp",
      "python_risk_calculation.cpp",
      "polymorphic_conversion.cpp"
  ],
  hdrs = [
      "python_planner_uct.hpp",
      "python_risk_calculation.hpp",
      "polymorphic_conversion.hpp",
  ],
  deps = [
    "@bark_project//bark/python_wrapper:common",
    "//bark_mcts/models/behavior:planner_uct_single_agent",
 #   "//bark/models/behavior:planner_uct_single_agent_macro_actions",
    "//bark_mcts/models/behavior:planner_uct_hypothesis",
    "//bark_mcts/models/behavior:planner_uct_cooperative",
    "//bark_mcts/models/behavior:planner_uct_risk_constraint",
    "//bark_mcts/models/behavior:planner_uct_nheuristic_risk_constraint",
    "//bark_mcts/models/behavior/belief_calculator:belief_calculator",
    "@bark_ml_project//bark_ml/python_wrapper:polymorphic_conversion",
    "@python_linux//:python-lib",
    "@pybind11//:pybind11",
    "@com_github_eigen_eigen//:eigen"
    ],
  visibility = ["//visibility:public"],
)

