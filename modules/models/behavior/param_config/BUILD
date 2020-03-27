
cc_library(
    name = "param_config",
    hdrs = [
            "mcts_parameters_from_param_server.hpp", 
            ],
    copts = ["-Iexternal/bark_project"],
    deps= [
        "@bark_project//modules:include",
        "@mamcts_project//mcts:mamcts",
    ],
    visibility = ["//visibility:public"],
)
