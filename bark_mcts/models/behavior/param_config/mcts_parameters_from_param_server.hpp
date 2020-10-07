// Copyright (c) 2020 Julian Bernhard
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef MCTS_PARAMETERS_FROM_PARAMETER_SERVER_HPP_
#define MCTS_PARAMETERS_FROM_PARAMETER_SERVER_HPP_

#include "bark/commons/params/params.hpp"
#include "mcts/mcts_parameters.h"

namespace bark{
namespace models {
namespace behavior {

inline mcts::MctsParameters::HypothesisBeliefTrackerParameters BeliefTrackerParametersFromParamServer(const commons::ParamsPtr& params,
                    std::unordered_map<unsigned int, unsigned int> fixed_hypothesis_set = {}) {
    mcts::MctsParameters::HypothesisBeliefTrackerParameters belief_tracker_parameters;

    belief_tracker_parameters.RANDOM_SEED_HYPOTHESIS_SAMPLING = params->GetInt("BeliefTracker::RandomSeedHypSampling", "Seed for hypothesis sampling", 2000);
    belief_tracker_parameters.HISTORY_LENGTH = params->GetInt("BeliefTracker::HistoryLength", "Length of probability history", 10);
    belief_tracker_parameters.PROBABILITY_DISCOUNT = params->GetReal("BeliefTracker::ProbabilityDiscount", "Discount factor for probabilities", 0.7f);
    belief_tracker_parameters.POSTERIOR_TYPE = params->GetInt("BeliefTracker::PosteriorType", "Zero for product, One for sum", 1);
    belief_tracker_parameters.FIXED_HYPOTHESIS_SET = fixed_hypothesis_set;

    return belief_tracker_parameters;
}


inline mcts::MctsParameters MctsParametersFromParamServer(const commons::ParamsPtr& params,
                    std::unordered_map<unsigned int, unsigned int> fixed_hypothesis_set = {}) {
    mcts::MctsParameters parameters;
    parameters.DISCOUNT_FACTOR = params->GetReal("Mcts::DiscountFactor", "Discount factor used in MDP problem", 0.9);
    parameters.RANDOM_SEED = params->GetInt("Mcts::RandomSeed", "Random seed applied used during search process", 1000);
    parameters.MAX_SEARCH_TIME = params->GetInt("Mcts::MaxSearchTime", "Maximum search time in milliseconds", 2000);
    parameters.MAX_NUMBER_OF_ITERATIONS = params->GetInt("Mcts::MaxNumIterations", "Maximum search time in milliseconds", 2000);
    parameters.MAX_SEARCH_DEPTH = params->GetInt("Mcts::MaxSearchDepth", "Maximum search tree depth", 1000);
    
    parameters.random_heuristic.MAX_SEARCH_TIME = params->GetInt("Mcts::RandomHeuristic::MaxSearchTime",
                                                         "Maximum time available for random rollout in milliseconds", 10);
    parameters.random_heuristic.MAX_NUMBER_OF_ITERATIONS = params->GetInt("Mcts::RandomHeuristic::MaxNumIterations",
                                                         "Maximum number of environment steps performed by random heuristic", 1000);

    parameters.uct_statistic.LOWER_BOUND = params->GetReal("Mcts::ReturnLowerBound", "Lower return bound used for normalization in UCT Statistic", -1000);
    parameters.uct_statistic.UPPER_BOUND = params->GetReal("Mcts::ReturnUpperBound", "Upper return bound used for normalization in UCT Statistic", 100);
    parameters.uct_statistic.EXPLORATION_CONSTANT = params->GetReal("Mcts::UctStatistic::ExplorationConstant", "Exploration constant of UCT", 0.7);
    parameters.uct_statistic.PROGRESSIVE_WIDENING_K = params->GetReal("Mcts::UctStatistic::ProgressiveWidening::K", "Upper return bound used for normalization in UCT Statistic", 1);
    parameters.uct_statistic.PROGRESSIVE_WIDENING_ALPHA = params->GetReal("Mcts::UctStatistic::ProgressiveWidening::Alpha", "Upper return bound used for normalization in UCT Statistic", 0.1);

    parameters.hypothesis_statistic.COST_BASED_ACTION_SELECTION = params->GetBool("Mcts::HypothesisStatistic::CostBasedActionSelection", "True, if costs instead of rewards are used for action selection", false);
    parameters.hypothesis_statistic.LOWER_COST_BOUND = params->GetReal("Mcts::LowerCostBound", "Upper cost bound used for normalization in UCT Statistic", 0);
    parameters.hypothesis_statistic.UPPER_COST_BOUND = params->GetReal("Mcts::UpperCostBound", "Lower cost bound used for normalization in UCT Statistic", 1);
    parameters.hypothesis_statistic.PROGRESSIVE_WIDENING_HYPOTHESIS_BASED = params->GetBool("Mcts::HypothesisStatistic::ProgressiveWidening::HypothesisBased",
                                                         "True, if progressive widening is calculated separately for hypothesis", true);
    parameters.hypothesis_statistic.PROGRESSIVE_WIDENING_ALPHA = params->GetReal("Mcts::HypothesisStatistic::ProgressiveWidening::Alpha", "Alpha used for prog. widening", 0.25);
    parameters.hypothesis_statistic.PROGRESSIVE_WIDENING_K = params->GetReal("Mcts::HypothesisStatistic::ProgressiveWidening::K", "K used for prog. widening", 4);
    parameters.hypothesis_statistic.EXPLORATION_CONSTANT = params->GetReal("Mcts::HypothesisStatistic::ExplorationConstant", "Exploration constant", 0.7);

    parameters.hypothesis_belief_tracker = BeliefTrackerParametersFromParamServer(params->AddChild("Mcts"));

    parameters.cost_constrained_statistic.LAMBDA = params->GetReal("Mcts::CostConstrainedStatistic::LambdaInit", "Initial lambda value", 2.0f);
    parameters.cost_constrained_statistic.COST_UPPER_BOUND = parameters.hypothesis_statistic.UPPER_COST_BOUND;
    parameters.cost_constrained_statistic.COST_LOWER_BOUND = parameters.hypothesis_statistic.LOWER_COST_BOUND;
    parameters.cost_constrained_statistic.REWARD_UPPER_BOUND = parameters.uct_statistic.UPPER_BOUND;
    parameters.cost_constrained_statistic.REWARD_LOWER_BOUND = parameters.uct_statistic.LOWER_BOUND;
    parameters.cost_constrained_statistic.COST_CONSTRAINT = 0.0f; // Set dynamically
    parameters.cost_constrained_statistic.KAPPA = params->GetReal("Mcts::CostConstrainedStatistic::Kappa", "Exploration constant", 10.0f);
    parameters.cost_constrained_statistic.GRADIENT_UPDATE_STEP = params->GetReal("Mcts::CostConstrainedStatistic::GradientUpdateScaling", "Update step scaling factor", 1.0f);
    parameters.cost_constrained_statistic.TAU_GRADIENT_CLIP = params->GetReal("Mcts::CostConstrainedStatistic::TauGradientClip", "Values smaller than one increase allowed gradient", 1.0f);
    parameters.cost_constrained_statistic.ACTION_FILTER_FACTOR = params->GetReal("Mcts::CostConstrainedStatistic::ActionFilterFactor",
               "Scales node counts in relation to value differences, favoring less visited nodes", 0.5f);

    auto float_to_bool_vec = [](const std::vector<float>& float_vec) {
        std::vector<bool> bool_vec(float_vec.size());
        std::transform(float_vec.begin(), float_vec.end(), bool_vec.begin(), [](const float& v){return v == 1.0 ? true : false;});
        return bool_vec;
    };
    parameters.cost_constrained_statistic.USE_COST_THRESHOLDING = float_to_bool_vec(params->GetListFloat("Mcts::CostConstrainedStatistic::UseCostTresholding",
               "Specify 1.0 if cost thresholding enabled for cost index", {0.0f, 0.0f}));
    parameters.cost_constrained_statistic.USE_CHANCE_CONSTRAINED_UPDATES = float_to_bool_vec(params->GetListFloat("Mcts::CostConstrainedStatistic::UseChanceConstrainedUpdate",
               "Track violations instead of cumulative cost during backpropagation", {0.0f, 0.0f}));
    parameters.cost_constrained_statistic.COST_THRESHOLDS = params->GetListFloat("Mcts::CostConstrainedStatistic::CostThresholds",
               "Cost thresholds for thresholded cost constraining", {0.1f, 0.0f});
    parameters.cost_constrained_statistic.USE_LAMBDA_POLICY = params->GetBool("Mcts::CostConstrainedStatistic::UseLambdaPolicy",
               "Lambda policy applied to first cost entry?", true);

    return parameters;
}


} // namespace behavior
} // namespace models
} // namespace bark

#endif // MCTS_PARAMETERS_FROM_PARAMETER_SERVER_HPP_