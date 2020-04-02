// Copyright (c) 2019 fortiss GmbH
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef MCTS_PARAMETERS_FROM_PARAMETER_SERVER_HPP_
#define MCTS_PARAMETERS_FROM_PARAMETER_SERVER_HPP_

#include "modules/commons/params/params.hpp"
#include "mcts/mcts_parameters.h"

namespace modules{
namespace models {
namespace behavior {


inline mcts::MctsParameters MctsParametersFromParamServer(const commons::ParamsPtr& params,
                    std::unordered_map<unsigned int, unsigned int> fixed_hypothesis_set = {}) {
    mcts::MctsParameters parameters;
    parameters.DISCOUNT_FACTOR = params->GetReal("Mcts::DiscountFactor", "Discount factor used in MDP problem", 0.9);
    parameters.RANDOM_SEED = params->GetInt("Mcts::RandomSeed", "Random seed applied used during search process", 1000);
    parameters.MAX_SEARCH_TIME = params->GetInt("Mcts::MaxSearchTime", "Maximum search time in milliseconds", 2000);
    parameters.MAX_NUMBER_OF_ITERATIONS = params->GetInt("Mcts::MaxNumIterations", "Maximum search time in milliseconds", 2000);
    
    parameters.random_heuristic.MAX_SEARCH_TIME = params->GetInt("Mcts::RandomHeuristic::MaxSearchTime",
                                                         "Maximum time available for random rollout in milliseconds", 10);
    parameters.random_heuristic.MAX_NUMBER_OF_ITERATIONS = params->GetInt("Mcts::RandomHeuristic::MaxNumIterations",
                                                         "Maximum number of environment steps performed by random heuristic", 1000);

    parameters.uct_statistic.LOWER_BOUND = params->GetReal("Mcts::UctStatistic::ReturnLowerBound", "Lower return bound used for normalization in UCT Statistic", -1000);
    parameters.uct_statistic.UPPER_BOUND = params->GetReal("Mcts::UctStatistic::ReturnUpperBound", "Upper return bound used for normalization in UCT Statistic", 100);
    parameters.uct_statistic.EXPLORATION_CONSTANT = params->GetReal("Mcts::UctStatistic::ExplorationConstant", "Exploration constant of UCT", 0.7);

    parameters.hypothesis_statistic.COST_BASED_ACTION_SELECTION = params->GetBool("Mcts::HypothesisStatistic::CostBasedActionSelection", "True, if costs instead of rewards are used for action selection", false);
    parameters.hypothesis_statistic.LOWER_COST_BOUND = params->GetReal("Mcts::HypothesisStatistic::LowerCostBound", "Upper cost bound used for normalization in UCT Statistic", 0);
    parameters.hypothesis_statistic.UPPER_COST_BOUND = params->GetReal("Mcts::HypothesisStatistic::UpperLowerBound", "Lower cost bound used for normalization in UCT Statistic", 1);
    parameters.hypothesis_statistic.PROGRESSIVE_WIDENING_HYPOTHESIS_BASED = params->GetBool("Mcts::HypothesisStatistic::ProgressiveWidening::HypothesisBased",
                                                         "True, if progressive widening is calculated separately for hypothesis", true);
    parameters.hypothesis_statistic.PROGRESSIVE_WIDENING_ALPHA = params->GetReal("Mcts::HypothesisStatistic::ProgressiveWidening::Alpha", "Alpha used for prog. widening", 0.5);
    parameters.hypothesis_statistic.PROGRESSIVE_WIDENING_K = params->GetReal("Mcts::HypothesisStatistic::ProgressiveWidening::K", "K used for prog. widening", 0.5);
    parameters.hypothesis_statistic.EXPLORATION_CONSTANT = params->GetReal("Mcts::HypothesisStatistic::ExplorationConstant", "Exploration constant", 0.7);

    parameters.hypothesis_belief_tracker.RANDOM_SEED_HYPOTHESIS_SAMPLING = params->GetInt("Mcts::BeliefTracker::RandomSeedHypSampling", "Seed for hypothesis sampling", 2000);
    parameters.hypothesis_belief_tracker.HISTORY_LENGTH = params->GetInt("Mcts::BeliefTracker::HistoryLength", "Length of probability history", 10);
    parameters.hypothesis_belief_tracker.PROBABILITY_DISCOUNT = params->GetReal("Mcts::BeliefTracker::ProbabilityDiscount", "Discount factor for probabilities", 0.7f);
    parameters.hypothesis_belief_tracker.POSTERIOR_TYPE = params->GetInt("Mcts::BeliefTracker::PosteriorType", "Zero for product, One for sum", 1);
    parameters.hypothesis_belief_tracker.FIXED_HYPOTHESIS_SET = fixed_hypothesis_set;

    return parameters;
}


} // namespace behavior
} // namespace models
} // namespace modules

#endif // MCTS_PARAMETERS_FROM_PARAMETER_SERVER_HPP_