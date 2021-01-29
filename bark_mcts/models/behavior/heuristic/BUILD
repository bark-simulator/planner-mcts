cc_library(
    name = "nn_heuristic",
    hdrs = [
            "nn_heuristic.hpp"
            ],
    copts = ["-Iexternal/bark_project"],
    deps= [
        "@bark_ml_project//bark_ml/library_wrappers/lib_fqf_iqn_qrdqn/model_loader:model_loader",
        "@bark_project//bark:include",
        "@mamcts_project//mcts:mamcts",
    ],
    visibility = ["//visibility:public"],
)

