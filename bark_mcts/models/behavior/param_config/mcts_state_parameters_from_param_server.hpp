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

inline EvaluationParameters MctsEvaluationParametersFromParamServer(const commons::ParamsPtr& params) {
    EvaluationParameters parameters(
                    params->GetBool("EvaluationParameters::AddSafeDist", "Calculate safe dist in evaluation", false),
                    params->GetBool("EvaluationParameters::StaticSafeDistIsTerminal", "Violating static safe dist gives terminal state", false),
                    params->GetBool("EvaluationParameters::DynamicSafeDistIsTerminal", "Violating dyn safe dist gives terminal state", false),
                    params->GetBool("EvaluationParameters::OutOfDrivableIsTerminal", "Violating driving corridor or driving out of map gives terminal state", false),
                    params->AddChild("EvaluatorParams"));
    return parameters;
}

inline StateParameters MctsStateParametersFromParamServer(const commons::ParamsPtr& params,
                    std::unordered_map<unsigned int, unsigned int> fixed_hypothesis_set = {}) {
    StateParameters parameters{
        params->GetReal("Mcts::State::GoalReward", "Reward for goal", 100.0f),
        params->GetReal("Mcts::State::CollisionReward", "Reward for goal", -1000.0f),
        params->GetReal("Mcts::State::SafeDistViolatedReward", "Reward for safe dist violation", -0.1f),
        params->GetReal("Mcts::State::DrivableCollisionReward", "Reward collision drivable", 0.0f),
        params->GetReal("Mcts::State::GoalCost", "Reward for goal", -100.0f),
        params->GetReal("Mcts::State::CollisionCost", "Reward for goal", 1000.0f),
        params->GetReal("Mcts::State::SafeDistViolatedCost", "Cost for safe dist violation", 0.1f),
        params->GetReal("Mcts::State::DrivableCollisionCost", "Cost for drivable area collision", 0.0f),
        params->GetReal("Mcts::State::CooperationFactor", "Cooperative MCTS, coop. factor", 0.2f),
        params->GetReal("Mcts::State::StepReward", "Reward given for each step in environment", 0.0f),
        params->GetReal("Mcts::State::PredictionK", "Reward given for each step in environment", 0.5f),
        params->GetReal("Mcts::State::PredictionAlpha", "Reward given for each step in environment", 0.0f),
        params->GetReal("Mcts::State::NormalizationTau", "time-based metrics are normalized to match prediction time", 0.2f),
        params->GetBool("Mcts::State::SplitSafeDistCollision", "Separate costs for safe dist violations", false),
        params->GetBool("Mcts::State::ChanceCosts", "Separate costs for safe dist violations", false),
        MctsEvaluationParametersFromParamServer(params->AddChild("Mcts::State"))
    };

    return parameters;
}




} // namespace behavior
} // namespace models
} // namespace bark

#endif // MCTS_PARAMETERS_FROM_PARAMETER_SERVER_HPP_