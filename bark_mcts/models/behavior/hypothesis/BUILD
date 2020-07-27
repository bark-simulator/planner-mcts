cc_library(
    name = "hypothesis",
    hdrs = [
        "behavior_hypothesis.hpp",
    ],
    deps = [
        "@bark_project//bark/models/behavior:behavior",
        "@bark_project//bark/commons/distribution:distribution",
        "//bark_mcts/models/behavior/risk_calculation:risk_calculation",
    ],
    visibility = ["//visibility:public"],
)

cc_library(
    name="include",
    hdrs=glob(["*.hpp"]),
    visibility = ["//visibility:public"],
)