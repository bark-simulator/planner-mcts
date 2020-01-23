load("@bazel_tools//tools/build_defs/repo:git.bzl", "git_repository")

def planner_uct_rules_dependencies():
    #local repository added like this
    _maybe(
    native.local_repository,
    name = "bark_project",
    path="/home/esterle/development/bark",
    )

    # _maybe(
    # git_repository,
    # name = "bark_project",
    # branch="master",
    # remote = "https://github.com/bark-simulator/bark",
    # )

    _maybe(
    git_repository,
    name = "mamcts_project",
    branch="master",
    remote = "https://github.com/juloberno/mamcts",
    )

def _maybe(repo_rule, name, **kwargs):
    if name not in native.existing_rules():
        repo_rule(name = name, **kwargs)