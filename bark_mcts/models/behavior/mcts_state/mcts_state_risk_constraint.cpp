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
                       const unsigned int& depth,
                       const std::unordered_map<mcts::AgentIdx, mcts::HypothesisId>& current_agents_hypothesis,
                       const std::vector<BehaviorModelPtr>& behavior_hypothesis,
                       const mcts::AgentIdx& ego_agent_id,
                       const StateParameters& state_parameters,
                       const std::unordered_map<mcts::AgentIdx, std::vector<mcts::Belief>>& current_hypothesis_beliefs,
                       const bark::commons::Probability& state_sequence_probability) :
                        MctsStateHypothesis(observed_world,
                                      is_terminal_state,
                                      num_ego_actions,
                                      depth,
                                      current_agents_hypothesis,
                                      behavior_hypothesis,
                                      ego_agent_id,
                                      state_parameters),
                        current_hypothesis_beliefs_(current_hypothesis_beliefs),
                        state_sequence_probability_(state_sequence_probability) {}

std::shared_ptr<MctsStateRiskConstraint> MctsStateRiskConstraint::clone() const {
  auto worldptr =
      std::dynamic_pointer_cast<ObservedWorld>(observed_world_->Clone());
  return std::make_shared<MctsStateRiskConstraint>(
      worldptr, is_terminal_state_, num_ego_actions_, depth_,
      current_agents_hypothesis_, behavior_hypotheses_,
      ego_agent_id_, state_parameters_, current_hypothesis_beliefs_, state_sequence_probability_);
}

std::shared_ptr<MctsStateRiskConstraint> MctsStateRiskConstraint::change_belief_reference(
                const std::unordered_map<mcts::AgentIdx, mcts::HypothesisId>& current_agents_hypothesis) const {
  auto worldptr =
      std::dynamic_pointer_cast<ObservedWorld>(observed_world_->Clone());
  return std::make_shared<MctsStateRiskConstraint>(
      worldptr, is_terminal_state_, num_ego_actions_, depth_,
      current_agents_hypothesis, behavior_hypotheses_,
      ego_agent_id_, state_parameters_, current_hypothesis_beliefs_, state_sequence_probability_);
}

std::shared_ptr<MctsStateRiskConstraint> MctsStateRiskConstraint::generate_next_state(const EvaluationResults& evaluation_results, const ObservedWorldPtr& predicted_world,
                                                        std::vector<mcts::Reward>& rewards,  mcts::EgoCosts& ego_cost) const {
  rewards.resize(this->get_num_agents(), 0.0f);
  rewards[this->ego_agent_idx] = reward_from_evaluation_results(evaluation_results, state_parameters_,
                                  this->calculate_prediction_time_span());
  
  ego_cost = ego_costs_from_evaluation_results(evaluation_results, state_parameters_,
                                  this->calculate_prediction_time_span());
  VLOG_IF_EVERY_N(5, ego_cost[0] != 0.0f || rewards[this->ego_agent_idx] != 0.0, 5) << "Ego reward: " << rewards[this->ego_agent_idx] << ", Ego cost: " << ego_cost[0];
  VLOG_IF_EVERY_N(5, rewards[this->ego_agent_idx] != 0.0f, 5) << "Ego reward: " << rewards[this->ego_agent_idx] << ", Ego cost: " << ego_cost;
  return std::make_shared<MctsStateRiskConstraint>(
      predicted_world, evaluation_results.is_terminal, num_ego_actions_, depth_+1,
      current_agents_hypothesis_, behavior_hypotheses_,
      ego_agent_id_, state_parameters_, current_hypothesis_beliefs_, state_sequence_probability_);
}

bark::commons::Probability MctsStateRiskConstraint::calculation_state_transition_probability(
              const ObservedWorld& to) const {
  bark::commons::Probability joint_probability = 1.0;
  for(const auto& agent : to.GetOtherAgents()) {
    const Action last_action = agent.second->GetBehaviorModel()->GetLastAction();
    bark::commons::Probability agent_action_probability = 0.0;
    for (std::size_t hyp_id = 0; hyp_id < behavior_hypotheses_.size(); ++hyp_id) {
      const auto last_action_prob = std::dynamic_pointer_cast<BehaviorHypothesis>(behavior_hypotheses_[hyp_id])
                                      ->GetProbability(last_action, *observed_world_, agent.second->GetAgentId());
      agent_action_probability  += last_action_prob * current_hypothesis_beliefs_.at(agent.second->GetAgentId()).at(hyp_id);
    }
    joint_probability *= agent_action_probability;
  }
  return joint_probability;
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
