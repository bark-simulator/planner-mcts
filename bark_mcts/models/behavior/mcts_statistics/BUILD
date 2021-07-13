cc_library(
    name = "mcts_statistics",
    hdrs = [
            "mcts_neural_cost_constrained_statistic.hpp"
            ],
    deps= [
        "@bark_ml_project//bark_ml/library_wrappers/lib_fqf_iqn_qrdqn/model_loader:model_loader",
        "@bark_ml_project//bark_ml/library_wrappers/lib_fqf_iqn_qrdqn/model/nn_to_value_converter:nn_to_value_converter",
        "@bark_ml_project//bark_ml/observers:include",
        "@bark_project//bark:include",
        "@mamcts_project//mcts:mamcts",
        "@torchcpp//:lib",
    ],
    visibility = ["//visibility:public"],
)

