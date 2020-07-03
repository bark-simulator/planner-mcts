// Copyright (c) 2019 fortiss GmbH
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef MCTS_PARAMETERS_MCTS_STATE_PARAMETERS_FROM_PARAMETER_SERVER_HPP_
#define MCTS_PARAMETERS_MCTS_STATE_PARAMETERS_FROM_PARAMETER_SERVER_HPP_

#include "bark/commons/params/params.hpp"
#include "bark/models/behavior/mcts_state/mcts_state_hypothesis.hpp"

#include "mcts/mcts_parameters.h"

namespace bark{
namespace models {
namespace behavior {


inline StateParameters MctsStateParametersFromParamServer(const commons::ParamsPtr& params,
                    std::unordered_map<unsigned int, unsigned int> fixed_hypothesis_set = {}) {
    StateParameters parameters;
    parameters.GOAL_REWARD = params->GetReal("Mcts::State::GoalReward", "Reward for goal", 100.0f);
    parameters.COLLISION_REWARD = params->GetReal("Mcts::State::CollisionReward", "Reward for goal", -1000.0f);

    parameters.GOAL_COST = params->GetReal("Mcts::State::GoalCost", "Reward for goal", -100.0f);
    parameters.COLLISION_COST = params->GetReal("Mcts::State::CollisionCost", "Reward for goal", 1000.0f);

    return parameters;
}


} // namespace behavior
} // namespace models
} // namespace bark

#endif // MCTS_PARAMETERS_FROM_PARAMETER_SERVER_HPP_