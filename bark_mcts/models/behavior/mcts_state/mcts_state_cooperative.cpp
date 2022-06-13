// Copyright (c) 2020 Julian Bernhard
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "bark_mcts/models/behavior/mcts_state/mcts_state_cooperative.hpp"

#include <memory>
#include <string>
#include <vector>
#include "bark/world/observed_world.hpp"

namespace bark {
namespace models {
namespace behavior {

using bark::world::ObservedWorld;
using bark::world::ObservedWorldPtr;
using bark::world::evaluation::EvaluatorCollisionEgoAgent;
using bark::world::evaluation::EvaluatorDrivableArea;
using bark::world::evaluation::EvaluatorGoalReached;

MctsStateCooperative::MctsStateCooperative(
                       const bark::world::ObservedWorldPtr& observed_world,
                       bool is_terminal_state,
                       const mcts::ActionIdx& num_ego_actions,
                       const float& prediction_time_span,
                       const mcts::AgentIdx& ego_agent_id,
                       const StateParameters& state_parameters) :
                        MctsStateBase<MctsStateCooperative>(observed_world,
                                      is_terminal_state,
                                      num_ego_actions,
                                      prediction_time_span,
                                      ego_agent_id,
                                      state_parameters,
                                      std::unordered_map<mcts::AgentIdx, mcts::HypothesisId>()){}

std::shared_ptr<MctsStateCooperative> MctsStateCooperative::clone() const {
    auto worldptr =
        std::dynamic_pointer_cast<ObservedWorld>(observed_world_->Clone());
    return std::make_shared<MctsStateCooperative>(
                    worldptr, is_terminal_state_, num_ego_actions_, depth_,
                    ego_agent_id_, state_parameters_); 
}

std::shared_ptr<MctsStateCooperative> MctsStateCooperative::execute(
    const mcts::JointAction& joint_action, std::vector<mcts::Reward>& rewards,
    mcts::EgoCosts& ego_cost) const {
  BARK_EXPECT_TRUE(!this->is_terminal());

  const auto predicted_world = predict(joint_action);

  return generate_next_state(predicted_world, rewards, ego_cost);
}

ObservedWorldPtr MctsStateCooperative::predict(const mcts::JointAction& joint_action) const {
  std::unordered_map<AgentId, DiscreteAction> agent_action_map;
  mcts::ActionIdx action_idx = 1;
  for (const auto& agent_idx : get_other_agent_idx()) {
    agent_action_map.insert({agent_idx, DiscreteAction(joint_action[action_idx])});
    action_idx++;
  }
  agent_action_map.insert({get_ego_agent_idx(), DiscreteAction(joint_action[this->ego_agent_idx])});
  auto predicted_world =
      std::dynamic_pointer_cast<ObservedWorld>(observed_world_->Predict(
          this->calculate_prediction_time_span(), agent_action_map));
  return predicted_world;
}

std::shared_ptr<MctsStateCooperative> MctsStateCooperative::generate_next_state(const ObservedWorldPtr& predicted_world,
                                                        std::vector<mcts::Reward>& rewards,  mcts::EgoCosts& ego_cost) const {
    const auto ego_evaluation_results = mcts_observed_world_evaluation(*predicted_world, state_parameters_.evaluation_parameters);
    const auto ego_agent_reward = reward_from_evaluation_results(ego_evaluation_results, state_parameters_,
                                  this->calculate_prediction_time_span());
    std::unordered_map<mcts::AgentIdx, mcts::Reward> other_agents_rewards;
    for(const auto& agent : predicted_world->GetOtherAgents()) {
      const auto others_observed_world = ObservedWorld(predicted_world, agent.first);
      const auto evaluation_results = mcts_observed_world_evaluation(others_observed_world, state_parameters_.evaluation_parameters);
      if (!evaluation_results.out_of_map) { // only count this agents reward  if still in map
        const auto reward = reward_from_evaluation_results(evaluation_results, state_parameters_,
                                  this->calculate_prediction_time_span());
        other_agents_rewards[agent.first] = reward;
      }
    }

    rewards.resize(this->get_num_agents(), 0.0f);
    mcts::Reward reward_sum = ego_agent_reward;
    for (const auto& agent_reward : other_agents_rewards) {
      reward_sum += agent_reward.second;
    }

    // Others rewards
    mcts::ActionIdx reward_idx = 1;
    rewards.resize(this->get_num_agents(), 0.0f);
    for (const auto& agent_idx : get_other_agent_idx()) {
      const auto& other_reward = other_agents_rewards[agent_idx];
      const auto rest_reward = reward_sum - other_reward;
      const auto cooperative_reward = 1.0/double(other_agents_rewards.size() + 1) *
             ((1 - state_parameters_.COOPERATION_FACTOR) * other_reward + state_parameters_.COOPERATION_FACTOR * rest_reward);
      rewards[reward_idx] = cooperative_reward;
      ++reward_idx;
       VLOG_IF(5, rewards[reward_idx] >= 0.1 || rewards[reward_idx] <= -0.1 ) << "Agent " << agent_idx << " r = " << rewards[reward_idx] << "reward sum" << reward_sum;
    }
   
    // Ego reward
    const auto rest_reward = reward_sum - ego_agent_reward;
    rewards[this->ego_agent_idx] = 1.0/double(other_agents_rewards.size() + 1) *
             ((1 - state_parameters_.COOPERATION_FACTOR) * ego_agent_reward + state_parameters_.COOPERATION_FACTOR * rest_reward);
    VLOG_IF(5, rewards[this->ego_agent_idx] >= 0.1 || rewards[this->ego_agent_idx] <= -0.1) << "Ego Agent r = " << rewards[this->ego_agent_idx] << "reward sum" << reward_sum;
    ego_cost.resize(this->get_num_costs(), 0.0f);
    ego_cost[0] = - rewards[this->ego_agent_idx];

    return std::make_shared<MctsStateCooperative>(
                    predicted_world, ego_evaluation_results.is_terminal, num_ego_actions_, depth_+1,
                    ego_agent_id_, state_parameters_);
}



}  // namespace behavior
}  // namespace models
}  // namespace bark
