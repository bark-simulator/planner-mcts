// Copyright (c) 2019 Julian Bernhard
// 
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.
// ========================================================

#ifndef BARK_MCTS_HYPOTHESIS_STATE_H_
#define BARK_MCTS_HYPOTHESIS_STATE_H_

// BARK
#include "bark/models/behavior/behavior_model.hpp"
#include "bark_mcts/models/behavior/action_store/behavior_action_store.hpp"
#include "bark_mcts/models/behavior/hypothesis/behavior_hypothesis.hpp"
#include "bark/models/behavior/motion_primitives/motion_primitives.hpp"
// MCTS Library
#include "bark_mcts/models/behavior/mcts_state/mcts_state_base.hpp"

namespace bark {
namespace world {
class ObservedWorld;
typedef std::shared_ptr<ObservedWorld> ObservedWorldPtr;
}  // namespace world
namespace models {
namespace behavior {

using BarkAction = bark::models::behavior::Action;

template<class T = void>
class MctsStateHypothesis : public MctsStateBase<MctsStateHypothesis<T>> {

  using MctsStateBase<MctsStateHypothesis<T>>::observed_world_;
  using MctsStateBase<MctsStateHypothesis<T>>::is_terminal_state_;
  using MctsStateBase<MctsStateHypothesis<T>>::num_ego_actions_;
  using MctsStateBase<MctsStateHypothesis<T>>::prediction_time_span_;
  using MctsStateBase<MctsStateHypothesis<T>>::other_agent_ids_;
  using MctsStateBase<MctsStateHypothesis<T>>::ego_agent_id_;
  using MctsStateBase<MctsStateHypothesis<T>>::state_parameters_;
  using MctsStateBase<MctsStateHypothesis<T>>::ego_behavior_model_;

public:
    MctsStateHypothesis(const bark::world::ObservedWorldPtr& observed_world,
                       bool is_terminal_state,
                       const mcts::ActionIdx& num_ego_actions,
                       const float& prediction_time_span,
                       const std::unordered_map<mcts::AgentIdx, mcts::HypothesisId>& current_agents_hypothesis,
                       const std::vector<BehaviorModelPtr>& behavior_hypothesis,
                       const BehaviorMotionPrimitivesPtr& ego_behavior_model,
                       const mcts::AgentIdx& ego_agent_id,
                       const StateParameters& state_parameters);

    std::shared_ptr<MctsStateHypothesis<T>> execute(const mcts::JointAction &joint_action,
                                            std::vector<mcts::Reward>& rewards,
                                            mcts::Cost& ego_cost) const;

    std::shared_ptr<MctsStateHypothesis<T>> clone() const;

    // Hypothesis State Interfaces
    mcts::ActionIdx plan_action_current_hypothesis(const mcts::AgentIdx& agent_idx) const;

    template<typename ActionType = bark::models::behavior::Action>
    mcts::Probability get_probability(const mcts::HypothesisId& hypothesis, const mcts::AgentIdx& agent_idx, 
                const ActionType& action) const; 

    template<typename ActionType = bark::models::behavior::Action>
    ActionType get_last_action(const mcts::AgentIdx& agent_idx) const;

    mcts::Probability get_prior(const mcts::HypothesisId& hypothesis, const mcts::AgentIdx& agent_idx) const;

    mcts::HypothesisId get_num_hypothesis(const mcts::AgentIdx& agent_idx) const;

    typedef BarkAction ActionType; // required for template-mechanism to compile

 protected:
  ObservedWorldPtr predict(const mcts::JointAction& joint_action) const;

  EvaluationResults evaluate(const ObservedWorld& observed_world) const;

  std::shared_ptr<MctsStateHypothesis<T>> generate_next_state(const EvaluationResults& evaluation_results, const ObservedWorldPtr& predicted_world) const;

  void calculate_ego_reward_cost(const EvaluationResults& evaluation_results, std::vector<mcts::Reward>& rewards,  mcts::Cost& ego_cost) const;


   const std::vector<BehaviorModelPtr>& behavior_hypotheses_;
  // const std::unordered_map<AgentIdx, std::vector<Belief>>& current_hypothesis_beliefs_;
  // const bark::commons::Probability state_sequence_probability_ // Probability to end in this state given the past expanded joint actions
   mutable std::unordered_map<AgentId, BehaviorModelPtr> behaviors_stored_;

};

}  // namespace behavior
}  // namespace models
}  // namespace bark

#include "bark_mcts/models/behavior/mcts_state/mcts_state_hypothesis.cpp"

#endif // BARK_MCTS_HYPOTHESIS_STATE_H_