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


inline mcts::MctsParameters MctsParametersFromParamServer(const commons::ParamsPtr& params) {
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

    parameters.uct_statistic.LOWER_BOUND = params->GetReal("Mcts::UctStatistic::ReturnLowerBound", "Lower return bound used for normalization in UCT Statistic", -1000);
    parameters.uct_statistic.UPPER_BOUND = params->GetReal("Mcts::UctStatistic::ReturnUpperBound", "Upper return bound used for normalization in UCT Statistic", 100);
    parameters.uct_statistic.EXPLORATION_CONSTANT = params->GetReal("Mcts::UctStatistic::ExplorationConstant", "Exploration constant of UCT", 0.7);

    return parameters;
}


} // namespace behavior
} // namespace models
} // namespace bark

#endif // MCTS_PARAMETERS_FROM_PARAMETER_SERVER_HPP_