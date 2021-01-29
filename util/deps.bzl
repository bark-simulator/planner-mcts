load("@bazel_tools//tools/build_defs/repo:git.bzl", "git_repository")

def planner_uct_rules_dependencies():
    _maybe(
    native.new_local_repository,
    name = "python_linux",
    path = "./bark_mcts/python_wrapper/venv/",
    build_file_content = """
cc_library(
    name = "python-lib",
    srcs = glob(["lib/libpython3.*", "libs/python3.lib", "libs/python36.lib"]),
    hdrs = glob(["include/**/*.h", "include/*.h"]),
    includes = ["include/python3.6m", "include", "include/python3.7m", "include/python3.5m"], 
    visibility = ["//visibility:public"],
)
"""
    )

    _maybe(
    native.local_repository,
    name = "bark_project",
   # commit="f126d8f7c6d31ffb3d3a722700b9cc7c224da0d5",
   # remote = "https://github.com/juloberno/bark",
    path="/home/julo/development/bark"
    )

    _maybe(
    native.local_repository,
    name = "bark_ml_project",
    #commit = "7e931df134e0525b2c466c721c849de4f2099ce2",
    #remote="https://github.com/juloberno/bark-ml"
    path = "/home/julo/development/bark-ml"
    )

    _maybe(
    native.new_local_repository,
    name = "torchcpp",
    path = "./bark_mcts/python_wrapper/venv/lib/python3.7/site-packages/",
    build_file_content = """
cc_library(
    name = "lib",
    srcs = ["torch/lib/libtorch.so",
                 "torch/lib/libc10.so", "torch/lib/libtorch_cpu.so"],
    hdrs = glob(["torch/include/**/*.h", "torch/include/*.h"]),
    visibility = ["//visibility:public"],
    linkopts = [
        "-ltorch",
        "-ltorch_cpu",
        "-lc10",
    ],
)
    """)

    _maybe(
    native.local_repository,
    name = "mamcts_project",
   # commit="9089e77b55f08a46816018cc8577a11322b3a32c",
   # remote = "https://github.com/juloberno/mamcts",
    path = "/home/julo/development/mamcts" 
    )

def _maybe(repo_rule, name, **kwargs):
    if name not in native.existing_rules():
        repo_rule(name = name, **kwargs)