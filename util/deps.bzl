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
    git_repository,
    name = "bark_project",
    commit="0c8f1b52bffcc639ac04ab944d41653f356eb1c8",
    remote = "https://github.com/bark-simulator/bark",
    )

    _maybe(
    git_repository,
    name = "bark_ml_project",
    commit = "84f9c74b60becbc4bc758e19b201d85a21880717",
    remote="https://github.com/bark-simulator/bark-ml"
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
    strip_include_prefix="torch/include/",
    visibility = ["//visibility:public"],
    linkopts = [
        "-ltorch",
        "-ltorch_cpu",
        "-lc10",
    ],
)
    """)

    _maybe(
    git_repository,
    name = "mamcts_project",
    commit="b3e65e7dcd508e43fd77eb75d89b89b3ea608fff",
    remote = "https://github.com/juloberno/mamcts",
    )

def _maybe(repo_rule, name, **kwargs):
    if name not in native.existing_rules():
        repo_rule(name = name, **kwargs)