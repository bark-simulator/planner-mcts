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

MctsStateHypothesis::MctsStateHypothesis(
                       const bark::world::ObservedWorldPtr& observed_world,
                       bool is_terminal_state,
                       const mcts::ActionIdx& num_ego_actions,
                       const float& prediction_time_span,
                       const std::unordered_map<mcts::AgentIdx, mcts::HypothesisId>& current_agents_hypothesis,
                       const std::vector<BehaviorModelPtr>& behavior_hypothesis,
                       const BehaviorMotionPrimitivesPtr& ego_behavior_model,
                       const mcts::AgentIdx& ego_agent_id,
                       const StateParameters& state_parameters) :
                        MctsStateBase<MctsStateHypothesis>(observed_world,
                                      is_terminal_state,
                                      num_ego_actions,
                                      prediction_time_span,
                                      ego_behavior_model,
                                      ego_agent_id,
                                      state_parameters,
                                      current_agents_hypothesis),
                        behavior_hypotheses_(behavior_hypothesis) {
    // start at index 1 since first agent is ego agent
    for(const auto& agent_id : other_agent_ids_) {
        behaviors_stored_[agent_id] = std::make_shared<BehaviorActionStore>(nullptr);
    }
}

std::shared_ptr<MctsStateHypothesis> MctsStateHypothesis::clone() const {
  auto worldptr =
      std::dynamic_pointer_cast<ObservedWorld>(observed_world_->Clone());
  return std::make_shared<MctsStateHypothesis>(
      worldptr, is_terminal_state_, num_ego_actions_, prediction_time_span_,
      current_agents_hypothesis_, behavior_hypotheses_, ego_behavior_model_,
      ego_agent_id_, state_parameters_);
}

std::shared_ptr<MctsStateHypothesis> MctsStateHypothesis::execute(
    const mcts::JointAction& joint_action, std::vector<mcts::Reward>& rewards,
    mcts::Cost& ego_cost) const {
  BARK_EXPECT_TRUE(!is_terminal());

  const auto predicted_world = predict(joint_action);

  EvaluationResults evaluation_results = evaluate(*predicted_world);

  calculate_ego_reward_cost(evaluation_results, rewards, ego_cost);

  return generate_next_state(evaluation_results, predicted_world);
}

ObservedWorldPtr MctsStateHypothesis::predict(const mcts::JointAction& joint_action) const {
  ego_behavior_model_->ActionToBehavior(DiscreteAction(joint_action[this->ego_agent_idx]));
  auto predicted_behaviors_ = behaviors_stored_;
  mcts::AgentIdx action_idx = 1;
  for (const auto& agent_id : get_other_agent_idx()) {
      std::dynamic_pointer_cast<BehaviorActionStore>(predicted_behaviors_[agent_id])
            ->MakeBehaviorActive(static_cast<ActionHash>(joint_action[action_idx]));
    action_idx++;
  }
  const auto predicted_world = 
            observed_world_->Predict(prediction_time_span_, ego_behavior_model_, predicted_behaviors_);
  return predicted_world;
}

EvaluationResults MctsStateHypothesis::evaluate(const ObservedWorld& observed_world) const {
  EvaluationResults evaluation_results;

  if (observed_world.GetEgoAgent()) {
    auto ego_id = observed_world.GetEgoAgent()->GetAgentId();
    auto evaluator_drivable_area = EvaluatorDrivableArea();
    auto evaluator_collision_ego = EvaluatorCollisionEgoAgent(ego_id);
    auto evaluator_goal_reached = EvaluatorGoalReached(ego_id);

    evaluation_results.collision_drivable_area =
        boost::get<bool>(evaluator_drivable_area.Evaluate(observed_world));
    evaluation_results.collision_other_agent =
        boost::get<bool>(evaluator_collision_ego.Evaluate(observed_world));
    evaluation_results.goal_reached =
        boost::get<bool>(evaluator_goal_reached.Evaluate(observed_world));
    evaluation_results.out_of_map = false;
  } else {
    evaluation_results.out_of_map = true;
  }
  evaluation_results.is_terminal = (evaluation_results.collision_drivable_area || evaluation_results.collision_other_agent
                                   || evaluation_results.goal_reached || evaluation_results.out_of_map);
  return evaluation_results;
}

std::shared_ptr<MctsStateHypothesis> MctsStateHypothesis::generate_next_state(const EvaluationResults& evaluation_results, const ObservedWorldPtr& predicted_world) const {
  return std::make_shared<MctsStateHypothesis>(
      predicted_world, evaluation_results.is_terminal, num_ego_actions_, prediction_time_span_,
      current_agents_hypothesis_, behavior_hypotheses_, ego_behavior_model_,
      ego_agent_id_, state_parameters_);
}

void MctsStateHypothesis::calculate_ego_reward_cost(const EvaluationResults& evaluation_results, std::vector<mcts::Reward>& rewards,  mcts::Cost& ego_cost) const {
  rewards.resize(this->get_num_agents(), 0.0f);
  rewards[this->ego_agent_idx] =
      (evaluation_results.collision_drivable_area || evaluation_results.collision_other_agent || evaluation_results.out_of_map) * state_parameters_.COLLISION_REWARD +
      evaluation_results.goal_reached * state_parameters_.GOAL_REWARD;

  ego_cost = (evaluation_results.collision_drivable_area || evaluation_results.collision_other_agent || evaluation_results.out_of_map) *state_parameters_.COLLISION_COST +
      evaluation_results.goal_reached * state_parameters_.GOAL_COST;
  VLOG_IF_EVERY_N(5, ego_cost != 0.0f, 3) << "Ego reward: " << rewards[this->ego_agent_idx] << ", Ego cost: " << ego_cost;
}

mcts::ActionIdx MctsStateHypothesis::plan_action_current_hypothesis(const mcts::AgentIdx& agent_idx) const {
    auto bark_agent_id = agent_idx;
    auto observed_world_for_other = observed_world_->ObserveForOtherAgent(bark_agent_id);
    const mcts::HypothesisId agt_hyp_id = this->current_agents_hypothesis_.at(agent_idx);
    const auto& trajectory = behavior_hypotheses_[agt_hyp_id]->Plan(prediction_time_span_, *observed_world_for_other);
    const BarkAction bark_action = behavior_hypotheses_[agt_hyp_id]->GetLastAction();
    const auto& behavior_status = behavior_hypotheses_[agt_hyp_id]->GetBehaviorStatus();
    const mcts::ActionIdx mcts_action = std::dynamic_pointer_cast<BehaviorActionStore>(
        behaviors_stored_[bark_agent_id])->Store(bark_action, trajectory, behavior_status);
    return mcts_action;
}

template<>
bark::models::behavior::Action MctsStateHypothesis::get_last_action(const mcts::AgentIdx& agent_idx) const {
    auto bark_agent_id = agent_idx;
    return observed_world_->GetAgent(bark_agent_id)->GetStateInputHistory().back().second;
}


template<>
mcts::Probability MctsStateHypothesis::get_probability(const mcts::HypothesisId& hypothesis, const mcts::AgentIdx& agent_idx, 
            const bark::models::behavior::Action& action) const {
    auto bark_agent_id = agent_idx;
    auto hypothesis_ptr = std::dynamic_pointer_cast<BehaviorHypothesis>(behavior_hypotheses_[hypothesis]);
    if(!hypothesis_ptr) {
        LOG(FATAL) << "Behaviors specified as hypothesis are not castable to hypothesis.";
    }
    auto prob = static_cast<mcts::Probability>(
        hypothesis_ptr->GetProbability(action, *observed_world_, bark_agent_id));
    return prob;
}

mcts::Probability MctsStateHypothesis::get_prior(const mcts::HypothesisId& hypothesis, const mcts::AgentIdx& agent_idx) const { return 0.5f;};

mcts::HypothesisId MctsStateHypothesis::get_num_hypothesis(const mcts::AgentIdx& agent_idx) const {return behavior_hypotheses_.size();};

}  // namespace behavior
}  // namespace models
}  // namespace bark
