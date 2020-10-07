// Copyright (c) 2020 Julian Bernhard
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef MCTS_PARAMETERS_MCTS_STATE_PARAMETERS_FROM_PARAMETER_SERVER_HPP_
#define MCTS_PARAMETERS_MCTS_STATE_PARAMETERS_FROM_PARAMETER_SERVER_HPP_

#include "bark/commons/params/params.hpp"

#include "mcts/mcts_parameters.h"

using bark::world::evaluation::SafeDistanceLabelFunction;

namespace bark{
namespace models {
namespace behavior {

inline EvaluationParameters MctsEvaluationParametersFromParamServer(const commons::ParamsPtr& params,
                    std::unordered_map<unsigned int, unsigned int> fixed_hypothesis_set = {}) {
    EvaluationParameters parameters{params->GetBool("EvaluationParameters::AddSafeDist", "Calculate safe dist in evaluation", false),
      SafeDistanceLabelFunction("safe_dist", 
        params->GetBool("EvaluationParameters::SafeDist::ToRear", "Include rear agent", true),
        params->GetReal("EvaluationParameters::SafeDist::ReactionTime", "Reward for goal", 100.0f),
        params->GetReal("EvaluationParameters::SafeDist::MaxEgoDecceleration", "Maximum ego decceleration", -5.0f),
        params->GetReal("EvaluationParameters::SafeDist::MaxOtherDecceleration", "Maximum other decceleration", -5.0f))};
    return parameters;
}

inline StateParameters MctsStateParametersFromParamServer(const commons::ParamsPtr& params,
                    std::unordered_map<unsigned int, unsigned int> fixed_hypothesis_set = {}) {
    StateParameters parameters{
        params->GetReal("Mcts::State::GoalReward", "Reward for goal", 100.0f),
        params->GetReal("Mcts::State::CollisionReward", "Reward for goal", -1000.0f),
        params->GetReal("Mcts::State::SafeDistViolatedReward", "Reward for safe dist violation", -1.0f),
        params->GetReal("Mcts::State::GoalCost", "Reward for goal", -100.0f),
        params->GetReal("Mcts::State::CollisionCost", "Reward for goal", 1000.0f),
        params->GetReal("Mcts::State::SafeDistViolatedCost", "Reward for goal", 1000.0f),
        params->GetReal("Mcts::State::CooperationFactor", "Cooperative MCTS, coop. factor", 0.2f),
        params->GetReal("Mcts::State::StepReward", "Reward given for each step in environment", 0.0f),
        params->GetBool("Mcts::State::SplitSafeDistCollision", "Separate costs for safe dist violations", false),
        MctsEvaluationParametersFromParamServer(params)
    };

    return parameters;
}




} // namespace behavior
} // namespace models
} // namespace bark

#endif // MCTS_PARAMETERS_FROM_PARAMETER_SERVER_HPP_