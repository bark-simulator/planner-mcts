load("@bazel_tools//tools/build_defs/repo:git.bzl", "git_repository")

def planner_uct_rules_dependencies():
    _maybe(
    native.local_repository,
    name = "bark_project",
    #commit = "e927e967c2e97cb60c0c123b5030ecd4bc6db68c",
    #remote = "https://github.com/bark-simulator/bark"
    path="/home/bernhard/development/bark",
    )

    _maybe(
    native.local_repository,
    name = "mamcts_project",
    #commit = "e927e967c2e97cb60c0c123b5030ecd4bc6db68c",
    #remote = "https://github.com/bark-simulator/bark"
    path="/home/bernhard/development/mamcts",
    )

def _maybe(repo_rule, name, **kwargs):
    if name not in native.existing_rules():
        repo_rule(name = name, **kwargs)