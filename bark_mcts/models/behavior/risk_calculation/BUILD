cc_library(
    name = "risk_calculation",
    hdrs = [
        "common.hpp",
        "prior_knowledge_function.hpp",
        "prior_knowledge_region.hpp",
        "scenario_risk_function.hpp",
        "prior_knowledge_function_definition.hpp"
    ],
    srcs = [
        "prior_knowledge_function.cpp",
        "prior_knowledge_region.cpp"
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