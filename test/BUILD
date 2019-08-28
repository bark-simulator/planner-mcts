cc_test(
    name = "single_agent_test",
    srcs = [
        "single_agent_test.cc",
    ],
    copts = ["-Iexternal/gtest/include"],
    deps = [
        "//src:planner_uct_single_agent",
        "@bark_project//modules/world:world",
        "@bark_project//modules/models/tests:make_test_world",
        "@gtest//:main",
    ],
)