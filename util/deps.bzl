load("@bazel_tools//tools/build_defs/repo:git.bzl", "git_repository")

def planner_uct_rules_dependencies():
    # _maybe(
    # git_repository,
    # name = "bark_project",
    # commit="042ed605f5631cfd59cc8dc9bc68325de4412d19",
    # remote = "https://github.com/bark-simulator/bark",
    # )

    _maybe(
    git_repository,
    name = "bark_project",
    branch="motion_primitives",
    remote = "https://github.com/bark-simulator/bark",
    )

    # _maybe(
    # native.local_repository,
    # name = "bark_project",
    # path = "/home/esterle/development/bark",
    # )

    _maybe(
    git_repository,
    name = "mamcts_project",
    commit="77d3efd13b508d6eb33f8fad1ba1665666529513",
    remote = "https://github.com/juloberno/mamcts",
    )

def _maybe(repo_rule, name, **kwargs):
    if name not in native.existing_rules():
        repo_rule(name = name, **kwargs)