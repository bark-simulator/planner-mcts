# Risk-Constrained, Probabilistic and Cooperative Behavior Planning using Monte Carlo Tree Search
![CI Build](https://github.com/bark-simulator/planner-mcts/workflows/CI/badge.svg)

Implementation of Monte Carlo Tree Search Planning (MCTS) Variants in C++ using the C++ MCTS library [mamcts](https://github.com/juloberno/mamcts). The implementations can be used as a behavior model in [BARK](https://github.com/bark-simulator/bark).

## Planning Variants
All planning variants are forms of multi-agent Monte Carlo Tree Search. The ego vehicle and other traffic participants independently select an action in the selection and expansion steps of the search. Their joint action determines the next environment state. The implemented planning variants are: 
 - **BehaviorUCTHypothesis**: A planner modeling game-theoretic planning using probabilistic predictions of other traffic participants. It employs belief tracking over behavior hypothesis partitioning a behavior space and root-belief-sampling. Different configurations enable robustness-based or stochastic action prediction of other traffic participants. Details are found in the publication
    [Robust Stochastic Bayesian Games for Behavior Space Coverage](https://arxiv.org/abs/2003.11281) (RSS Workshop, 2020)
- **BehaviorUCTCooperative**: This model selects actions for ego and other agents with UCT, thereby applying a combined
cooperative reward function that can be tuned with a cooperation factor.
- **BehaviorUCTRiskContraint**: Extends the BehaviorUCTHypothesis to employ a risk-constrained action selection for the ego agent. The resulting risk-constrained planner constrains the risk of violating a safety envelope. It achieves correspondence between the specified violation risk and the statistics of observed safety violations obtained from averaging safety violations over many benchmarked scenarios. Details are found in the publication
 [Risk-Constrained Interactive Safety under Behavior Uncertainty for Autonomous Driving](https://arxiv.org/abs/2102.03053) (IV 2021)
- **BehaviorUCTRiskContraintNeuralHeuristic**: Extends the BehaviorUCTRiskContraint to use warm-starting of return and risk action-values which accelerates the convergence of the search. The action values for warm-starting are inferred from a neural network trained offline with the BARK's learning framework [BARK-ML](https://github.com/bark-simulator/bark-ml).


## Configurable Variants of Decision-Theoretic Models:
The above implementations can be configured to model planning under different decision-theoretic models by 
- setting parameters accordingly 
- using a specific behavior hypothesis set to predict other traffic participants. Code to configure hypothesis sets is also provided in this repo.

The following models can be configured:
- **Markov Decision Process (MDP)**: Probabilistic prediction of other traffic participants using a single behavior hypothesis. This concept models stochastic environment transitions that are independent of prior states. 
- **Robust Markov Decision Process (RMDP)**: Probabilistic prediction of other traffic participants using a single behavior hypothesis. Yet, in contrast to the MDP, it estimates and selects traffic participants' behavior parameters, provoking worst-case outcomes for the ego vehicle.
- **Stochastic Bayesian Game (SBG)**: The [SBG](https://arxiv.org/abs/1506.01170) is a game-theoretic model combining Bayesian and stochastic games. It models irrational decisions of players in the game. This game-theoretic model can be configured by providing a set of behavior hypotheses, given as BARK BehaviorModels, to the planner BehaviorUCTHypothesis and configuring the belief tracker. 
- **Robust Stochastic Bayesian Game (RSBG)**: Game-theoretic model combining the SBG with an RMDP. The model employs worst-case considerations within the behavior hypothesis.
- **Omniscient**: The BehaviorUCTHypothesis planner can employ the behavior models used in the simulation for prediction during planning. This simulates the unrealizable case of having full knowledge of the behavior of other traffic participants and serves as an upper bound on the performance in a benchmark.

Further details on these configurations are given in the publications mentioned above. The separate repository [mcts_benchmark_repo](https://github.com/bark-simulator/example_benchmark_2) provides examples of the above configuration options for benchmarking. 

## Installation
### For testing
The repository [mcts_benchmark_repo](https://github.com/bark-simulator/example_benchmark_2) provides examples how to employ the planner variants in a benchmark.

### For Development
1. [Clone the repository](https://git.fortiss.org/bark-simulator/planner-mcts) and change to the base repository directory
2. `bash util/setup_test_venv.sh`: This will create a virtual python environment (located in ./bark_mcts/python_wrapper/venv)
2. `source util/into_test_venv.sh`: This will activate the virtual environment and set environment variables(keep this in mind for the future: each time you use Bazel, even beyond this installation, be sure to have run this command beforehand)
3. `bazel build //...`: This will build the whole library
4. `bazel test //...`: This will run all specified tests
