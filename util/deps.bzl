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
        commit="5e2ce8893e21f7236ee7c43351e61c0bce28a057",
        remote = "https://github.com/juloberno/bark",
    )

    _maybe(
    git_repository,
    name = "mamcts_project",
    commit="be897993bf2ba8cf0259d4c1b78c8a0f3fcde457",
    remote = "https://github.com/juloberno/mamcts",
    )

def _maybe(repo_rule, name, **kwargs):
    if name not in native.existing_rules():
        repo_rule(name = name, **kwargs)