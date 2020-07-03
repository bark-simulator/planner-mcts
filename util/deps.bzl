load("@bazel_tools//tools/build_defs/repo:git.bzl", "git_repository")

def planner_uct_rules_dependencies():
    _maybe(
    native.new_local_repository,
    name = "python_linux",
    path = "./bark/python_wrapper/venv/",
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
    commit="02e5c872393efd0798d665b4b80b82158dde77b9",
    remote = "https://github.com/bark-simulator/bark",
    )

    _maybe(
    git_repository,
    name = "mamcts_project",
    commit="6bbc33ddc95ce2b55a67a4d4b037f996e343eefc",
    remote = "https://github.com/juloberno/mamcts",
    )

def _maybe(repo_rule, name, **kwargs):
    if name not in native.existing_rules():
        repo_rule(name = name, **kwargs)