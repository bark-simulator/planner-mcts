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


mcts::MctsParameters MctsParametersFromParamServer(const commons::ParamsPtr& params) {
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

    parameters.hypothesis_statistic.COST_BASED_ACTION_SELECTION = false;
    parameters.hypothesis_statistic.LOWER_COST_BOUND = 0;
    parameters.hypothesis_statistic.UPPER_COST_BOUND = 1;
    parameters.hypothesis_statistic.PROGRESSIVE_WIDENING_HYPOTHESIS_BASED = true;
    parameters.hypothesis_statistic.PROGRESSIVE_WIDENING_ALPHA = 0.5;
    parameters.hypothesis_statistic.PROGRESSIVE_WIDENING_K = 1;
    parameters.hypothesis_statistic.EXPLORATION_CONSTANT = 0.7;

    parameters.hypothesis_belief_tracker.RANDOM_SEED_HYPOTHESIS_SAMPLING = 1000;
    parameters.hypothesis_belief_tracker.HISTORY_LENGTH = 4;
    parameters.hypothesis_belief_tracker.PROBABILITY_DISCOUNT = 1.0f;
    parameters.hypothesis_belief_tracker.POSTERIOR_TYPE = 0; // = HypothesisBeliefTracker::PRODUCT;
    parameters.hypothesis_belief_tracker.FIXED_HYPOTHESIS_SET = {};

    return parameters;
}


} // namespace behavior
} // namespace models
} // namespace modules

#endif // MCTS_PARAMETERS_FROM_PARAMETER_SERVER_HPP_