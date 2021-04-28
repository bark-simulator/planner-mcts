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
    commit="392055a5311f99f0e3dc414b6e05a1cb256d11b1",
    remote = "https://github.com/bark-simulator/bark",
    #path="/home/julo/development/bark"
    )

    _maybe(
    git_repository,
    name = "bark_ml_project",
    commit = "2a5c8b0d6a13f340d93d8bb7eb66f6a2559d2d6f",
    remote="https://github.com/juloberno/bark-ml"
    #path = "/home/julo/development/bark-ml"
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
    commit="2c61f35ff6df32ada7be59d98ff9a149bace6e58",
    remote = "https://github.com/juloberno/mamcts",
    #path = "/home/bernhard/development/mamcts" 
    )

def _maybe(repo_rule, name, **kwargs):
    if name not in native.existing_rules():
        repo_rule(name = name, **kwargs)