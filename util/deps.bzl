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
        commit="40a65589e9d51b5110326ffab019f92faf8ea42b",
        remote = "https://github.com/bark-simulator/bark",
    )
    
    _maybe(
    git_repository,
    name = "bark_ml",
    commit="82e7d75a54da61a79de3c580a333d72d6b2def41",
    remote = "https://github.com/SebastianGra/bark-ml_MCTS_RL",
    )


    _maybe(
    git_repository,
    name = "libtensorflow-RL-MCTS",
    commit="31adfc2112ce41b81574b6f44192ec47f0853fbc",
    remote = "https://github.com/steven-guo94/libtensorflow_so",
    )

    _maybe(
    git_repository,
    name = "mamcts_project",
    commit="c74286507ee34ca66e0b7b423f0367bd62ff7a90",
    remote = "https://github.com/juloberno/mamcts",
    )

def _maybe(repo_rule, name, **kwargs):
    if name not in native.existing_rules():
        repo_rule(name = name, **kwargs)

