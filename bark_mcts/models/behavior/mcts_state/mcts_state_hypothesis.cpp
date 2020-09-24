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

template <typename T>
MctsStateHypothesis<T>::MctsStateHypothesis(
                       const bark::world::ObservedWorldPtr& observed_world,
                       bool is_terminal_state,
                       const mcts::ActionIdx& num_ego_actions,
                       const float& prediction_time_span,
                       const std::unordered_map<mcts::AgentIdx, mcts::HypothesisId>& current_agents_hypothesis,
                       const std::vector<BehaviorModelPtr>& behavior_hypothesis,
                       const mcts::AgentIdx& ego_agent_id,
                       const StateParameters& state_parameters) :
                        MctsStateBase<MctsStateHypothesis<T>>(observed_world,
                                      is_terminal_state,
                                      num_ego_actions,
                                      prediction_time_span,
                                      ego_agent_id,
                                      state_parameters,
                                      current_agents_hypothesis),
                        behavior_hypotheses_(behavior_hypothesis) {
    // start at index 1 since first agent is ego agent
    for(const auto& agent_id : MctsStateBase<MctsStateHypothesis<T>>::other_agent_ids_) {
        behaviors_stored_[agent_id] = std::make_shared<BehaviorActionStore>(nullptr);
    }
}

template <typename T>
std::shared_ptr<MctsStateHypothesis<T>> MctsStateHypothesis<T>::clone() const {
    return impl_clone(std::is_same<T, void>{});
}

template <typename T>
auto MctsStateHypothesis<T>::execute(
    const mcts::JointAction& joint_action, std::vector<mcts::Reward>& rewards,
    mcts::Cost& ego_cost) const {
  BARK_EXPECT_TRUE(!this->is_terminal());

  const auto predicted_world = MctsStateHypothesis<T>::predict(joint_action);

  EvaluationResults evaluation_results = mcts_observed_world_evaluation(*predicted_world);

  return MctsStateHypothesis<T>::generate_next_state(evaluation_results, predicted_world, rewards, ego_cost);
}

template <typename T>
ObservedWorldPtr MctsStateHypothesis<T>::predict(const mcts::JointAction& joint_action) const {
  auto predicted_behaviors_ = behaviors_stored_;
  mcts::AgentIdx action_idx = 1;
  for (const auto& agent_id : this->get_other_agent_idx()) {
      std::dynamic_pointer_cast<BehaviorActionStore>(predicted_behaviors_[agent_id])
            ->MakeBehaviorActive(static_cast<ActionHash>(joint_action[action_idx]));
    action_idx++;
  }
  auto ego_behavior = observed_world_->GetEgoAgent()->GetBehaviorModel()->Clone();
  ego_behavior->ActionToBehavior(DiscreteAction(joint_action[this->ego_agent_idx]));
  const auto predicted_world = 
            observed_world_->Predict(prediction_time_span_, ego_behavior, predicted_behaviors_);
  return predicted_world;
}

template <typename T>
auto MctsStateHypothesis<T>::generate_next_state(const EvaluationResults& evaluation_results, const ObservedWorldPtr& predicted_world,
                                                        std::vector<mcts::Reward>& rewards,  mcts::Cost& ego_cost) const {
  return impl_generate_next_state(std::is_same<T, void>{}, evaluation_results, predicted_world, rewards, ego_cost);
}

template <typename T>
mcts::ActionIdx MctsStateHypothesis<T>::plan_action_current_hypothesis(const mcts::AgentIdx& agent_idx) const {
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
template<>
inline bark::models::behavior::Action MctsStateHypothesis<void>::get_last_action(const mcts::AgentIdx& agent_idx) const {
    auto bark_agent_id = agent_idx;
    return observed_world_->GetAgent(bark_agent_id)->GetStateInputHistory().back().second;
}

template<>
template<>
inline mcts::Probability MctsStateHypothesis<void>::get_probability(const mcts::HypothesisId& hypothesis, const mcts::AgentIdx& agent_idx, 
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

class MctsStateRiskConstraint;
template<>
template<>
inline bark::models::behavior::Action MctsStateHypothesis<MctsStateRiskConstraint>::get_last_action(const mcts::AgentIdx& agent_idx) const {
    auto bark_agent_id = agent_idx;
    return observed_world_->GetAgent(bark_agent_id)->GetStateInputHistory().back().second;
}

template<>
template<>
inline mcts::Probability MctsStateHypothesis<MctsStateRiskConstraint>::get_probability(const mcts::HypothesisId& hypothesis, const mcts::AgentIdx& agent_idx, 
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

template <typename T>
mcts::Probability MctsStateHypothesis<T>::get_prior(const mcts::HypothesisId& hypothesis, const mcts::AgentIdx& agent_idx) const { return 0.5f;};

template <typename T>
mcts::HypothesisId MctsStateHypothesis<T>::get_num_hypothesis(const mcts::AgentIdx& agent_idx) const {return behavior_hypotheses_.size();};

}  // namespace behavior
}  // namespace models
}  // namespace bark
