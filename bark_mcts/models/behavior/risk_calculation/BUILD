cc_library(
    name = "risk_calculation",
    hdrs = [
        "prior_knowledge_function.hpp",
        "prior_knowledge_region.hpp",
        "scenario_risk_function.hpp",
    ],
    deps = [
        "@bark_project//bark/commons/params:params"
    ],
    visibility = ["//visibility:public"],
)

cc_library(
    name="include",
    hdrs=glob(["*.hpp"]),
    visibility = ["//visibility:public"],
)