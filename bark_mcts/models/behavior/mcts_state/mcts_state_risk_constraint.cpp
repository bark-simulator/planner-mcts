// Copyright (c) 2020 Julian Bernhard
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "bark_mcts/models/behavior/mcts_state/mcts_state_risk_constraint.hpp"

#include <memory>
#include <string>
#include <vector>
#include "bark/world/evaluation/evaluator_collision_ego_agent.hpp"
#include "bark/world/evaluation/evaluator_drivable_area.hpp"
#include "bark/world/evaluation/evaluator_goal_reached.hpp"
#include "bark/world/observed_world.hpp"

namespace bark {
namespace models {
namespace behavior {

using bark::world::ObservedWorld;
using bark::world::ObservedWorldPtr;
using bark::world::evaluation::EvaluatorCollisionEgoAgent;
using bark::world::evaluation::EvaluatorDrivableArea;
using bark::world::evaluation::EvaluatorGoalReached;

MctsStateRiskConstraint::MctsStateRiskConstraint(
                       const bark::world::ObservedWorldPtr& observed_world,
                       bool is_terminal_state,
                       const mcts::ActionIdx& num_ego_actions,
                       const float& prediction_time_span,
                       const std::unordered_map<mcts::AgentIdx, mcts::HypothesisId>& current_agents_hypothesis,
                       const std::vector<BehaviorModelPtr>& behavior_hypothesis,
                       const BehaviorMotionPrimitivesPtr& ego_behavior_model,
                       const mcts::AgentIdx& ego_agent_id,
                       const StateParameters& state_parameters,
                       const std::unordered_map<mcts::AgentIdx, std::vector<mcts::Belief>>& current_hypothesis_beliefs,
                       const bark::commons::Probability& state_sequence_probability) :
                        MctsStateHypothesis(observed_world,
                                      is_terminal_state,
                                      num_ego_actions,
                                      prediction_time_span,
                                      current_agents_hypothesis,
                                      behavior_hypothesis,
                                      ego_behavior_model,
                                      ego_agent_id,
                                      state_parameters),
                        current_hypothesis_beliefs_(current_hypothesis_beliefs),
                        state_sequence_probability_(state_sequence_probability) {}

std::shared_ptr<MctsStateRiskConstraint> MctsStateRiskConstraint::clone() const {
  auto worldptr =
      std::dynamic_pointer_cast<ObservedWorld>(observed_world_->Clone());
  return std::make_shared<MctsStateRiskConstraint>(
      worldptr, is_terminal_state_, num_ego_actions_, prediction_time_span_,
      current_agents_hypothesis_, behavior_hypotheses_, ego_behavior_model_,
      ego_agent_id_, state_parameters_, current_hypothesis_beliefs_, state_sequence_probability_);
}

std::shared_ptr<MctsStateRiskConstraint> MctsStateRiskConstraint::generate_next_state(const EvaluationResults& evaluation_results, const ObservedWorldPtr& predicted_world,
                                                        std::vector<mcts::Reward>& rewards,  mcts::Cost& ego_cost) const {
  const auto next_state_sequence_probability = state_sequence_probability_ * calculation_state_transition_probability(*predicted_world);
  return std::make_shared<MctsStateRiskConstraint>(
      predicted_world, evaluation_results.is_terminal, num_ego_actions_, prediction_time_span_,
      current_agents_hypothesis_, behavior_hypotheses_, ego_behavior_model_,
      ego_agent_id_, state_parameters_, current_hypothesis_beliefs_, next_state_sequence_probability);
}

bark::commons::Probability MctsStateRiskConstraint::calculation_state_transition_probability(
              const ObservedWorld& to) const {
  bark::commons::Probability probability = 1.0;
  for(const auto& agent : to.GetOtherAgents()) {
    const Action last_action = agent.second->GetBehaviorModel()->GetLastAction();
    for (const auto& hypothesis : behavior_hypotheses_) {
      const auto last_action_prob = std::dynamic_pointer_cast<BehaviorHypothesis>(hypothesis)
                                        ->GetProbability(last_action, *observed_world_, agent.second->GetAgentId());
      probability *= last_action_prob; // Todo add hypothesis belief weighting
    }
  }
}


bark::commons::Probability MctsStateRiskConstraint::calculate_sequence_probability(const ObservedWorld& to) const {
  return get_state_sequence_probability() * calculation_state_transition_probability(to);
}

bark::commons::Probability MctsStateRiskConstraint::get_state_sequence_probability() const {
  return state_sequence_probability_;
}



}  // namespace behavior
}  // namespace models
}  // namespace bark
