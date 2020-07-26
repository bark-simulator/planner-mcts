load("@bazel_tools//tools/build_defs/repo:git.bzl", "git_repository")

def planner_uct_rules_dependencies():
    _maybe(
    git_repository,
    name = "bark_project",
    commit="374f6848c2f0065926e586558f1257c35639166d",
    remote = "https://github.com/Lizhu-Chen/bark",
    )
    
    _maybe(
    git_repository,
    name = "bark_ml",
    commit="b83d87926f31f0f0e1ca6dfa52c4318e2c0a6f73",
    remote = "https://github.com/SebastianGra/bark-ml_MCTS_RL",
    )

    _maybe(
    git_repository,
    name = "libtensorflow-RL-MCTS",
    commit="84845b655f7bf6d3263dcddf465446edb192e21b",
    remote = "https://github.com/steven-guo94/libtensorflow_so",
    )

    _maybe(
    git_repository,
    name = "mamcts_project",
    commit="55d3478b954287fa360329cc8320855894b02ce5",
    remote = "https://github.com/juloberno/mamcts",
    )

def _maybe(repo_rule, name, **kwargs):
    if name not in native.existing_rules():
        repo_rule(name = name, **kwargs)

