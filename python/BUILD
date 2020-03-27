package(default_visibility = ["//visibility:public"])

cc_library(
  name = "planner_uct",
  srcs = ["python_planner_uct.cpp"],
  hdrs = ["python_planner_uct.hpp"],
  deps = [
    "@bark_project//python:common",
    "//modules/models/behavior:planner_uct_single_agent",
    "//modules/models/behavior:planner_uct_single_agent_macro_actions",
    "//modules/models/behavior:planner_uct_hypothesis",
    "@python_linux//:python-lib",
    "@pybind11//:pybind11",
    "@com_github_eigen_eigen//:eigen", 
  ],
  visibility = ["//visibility:public"],
)

