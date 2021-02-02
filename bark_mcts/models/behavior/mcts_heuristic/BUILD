cc_library(
    name = "mcts_heuristic",
    hdrs = [
            "mcts_neural_heuristic.hpp"
            ],
    deps= [
        "@bark_ml_project//bark_ml/library_wrappers/lib_fqf_iqn_qrdqn/model_loader:model_loader",
        "@bark_ml_project//bark_ml/observers:include",
        "@bark_project//bark:include",
        "@mamcts_project//mcts:mamcts",
        "@torchcpp//:lib",
    ],
    copts = ["-Iexternal/torchcpp/torch/include",
           "-Lexternal/torchcpp/torch/lib",
           "-D_GLIBCXX_USE_CXX11_ABI=0"],
    linkopts = ["-ltorch_cpu","-lc10", "-ltorch"],
    visibility = ["//visibility:public"],
)

