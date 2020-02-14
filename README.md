# planner-mcts
![CI Build](https://github.com/bark-simulator/planner-mcts/workflows/CI/badge.svg)
Behavior planners based on single- and multi-agent Monte Carlo Tree Search

### For Development
1. [Clone the repository](https://git.fortiss.org/bark-simulator/planner-mcts), change to base repository directory; change the .bazelrc: build --action_env CC=/usr/bin/gcc-7 --cxxopt='-std=c++14'
2. `virtualenv -p python3 ./python/venv`: this will create a virtual python environment (located in python/venv)
2. `source python/venv/bin/activate`: this will activate the virtual environment (keep this in mind for the future: each time you use Bazel, even beyond this installation, be sure to have run this command beforehand)
3. `bazel build //...`: this will build the whole library
4. `bazel test //...`: this will run all specified tests
