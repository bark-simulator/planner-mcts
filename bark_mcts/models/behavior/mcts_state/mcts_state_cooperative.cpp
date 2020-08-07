// Copyright (c) 2020 Julian Bernhard
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "bark_mcts/models/behavior/mcts_state/mcts_state_hypothesis.hpp"

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

MctsStateCooperative::MctsStateCooperative(
                       const bark::world::ObservedWorldPtr& observed_world,
                       bool is_terminal_state,
                       const mcts::ActionIdx& num_ego_actions,
                       const float& prediction_time_span,
                       const std::vector<BehaviorModelPtr>& behavior_hypothesis,
                       const BehaviorMotionPrimitivesPtr& ego_behavior_model,
                       const mcts::AgentIdx& ego_agent_id,
                       const StateParameters& state_parameters) :
                        MctsStateBase<MctsStateHypothesis<T>>(observed_world,
                                      is_terminal_state,
                                      num_ego_actions,
                                      prediction_time_span,
                                      ego_behavior_model,
                                      ego_agent_id,
                                      state_parameters,
                                      std::unordered_map<mcts::AgentIdx, mcts::HypothesisId>()){}

std::shared_ptr<MctsStateCooperative> MctsStateHypothesis<T>::clone() const {
    return 
}

auto MctsStateCooperative::execute(
    const mcts::JointAction& joint_action, std::vector<mcts::Reward>& rewards,
    mcts::Cost& ego_cost) const {
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
  agent_action_map.insert({get_ego_agent_idx(), DiscreteAction(joint_action[S::ego_agent_idx])});
  auto predicted_world =
      std::dynamic_pointer_cast<ObservedWorld>(observed_world_->Predict(
          prediction_time_span_, agent_action_map));
  return predicted_world;
}

auto MctsStateCooperative::generate_next_state(const ObservedWorldPtr& predicted_world,
                                                        std::vector<mcts::Reward>& rewards,  mcts::Cost& ego_cost) const {
    const auto ego_evaluation_results = mcts_observed_world_evaluation(others_observed_world);
    const ego_agent_reward = reward_from_evaluation_results(ego_evaluation_results, state_parameters_);
    std::unordered_map<mcts::AgentIdx, mcts::Reward> other_agents_rewards;
    for(const auto& agent : predicted_world->GetOtherAgents()) {
      const auto others_observed_world = ObservedWorld(*predicted_world, agent->GetAgentId());
      const auto evaluation_results = mcts_observed_world_evaluation(others_observed_world);
      const auto reward = reward_from_evaluation_results(evaluation_results, state_parameters_);
      agents_rewards[agent->GetAgentId()] = reward;
    }

    rewards.resize(this->get_num_agents(), 0.0f);
    mcts::Reward reward_sum = ego_agent_reward;
    for (const auto& agent_reward : agent_rewards) {
      reward_sum += agent_reward.second;
    }

    mcts::ActionIdx = 1
    for (auto& )
}



}  // namespace behavior
}  // namespace models
}  // namespace bark
