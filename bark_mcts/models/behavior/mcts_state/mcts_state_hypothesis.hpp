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
  protected:
    using MctsStateBase<MctsStateHypothesis<T>>::observed_world_;
    using MctsStateBase<MctsStateHypothesis<T>>::is_terminal_state_;
    using MctsStateBase<MctsStateHypothesis<T>>::num_ego_actions_;
    using MctsStateBase<MctsStateHypothesis<T>>::prediction_time_span_;
    using MctsStateBase<MctsStateHypothesis<T>>::other_agent_ids_;
    using MctsStateBase<MctsStateHypothesis<T>>::ego_agent_id_;
    using MctsStateBase<MctsStateHypothesis<T>>::state_parameters_;

public:
    MctsStateHypothesis(const bark::world::ObservedWorldPtr& observed_world,
                       bool is_terminal_state,
                       const mcts::ActionIdx& num_ego_actions,
                       const float& prediction_time_span,
                       const std::unordered_map<mcts::AgentIdx, mcts::HypothesisId>& current_agents_hypothesis,
                       const std::vector<BehaviorModelPtr>& behavior_hypothesis,
                       const mcts::AgentIdx& ego_agent_id,
                       const StateParameters& state_parameters);

    
    auto execute(const mcts::JointAction &joint_action,
                                            std::vector<mcts::Reward>& rewards,
                                            mcts::EgoCosts& ego_cost) const;

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

  mcts::ActionIdx get_num_actions(mcts::AgentIdx agent_idx) const { 
    if(agent_idx == this->get_ego_agent_idx()) {
        return num_ego_actions_;
    } else {
      return std::numeric_limits<mcts::ActionIdx>::max();
    }
  }

 protected:
    ObservedWorldPtr predict(const mcts::JointAction& joint_action) const;

    EvaluationResults evaluate(const ObservedWorld& observed_world) const;

    auto generate_next_state(const EvaluationResults& evaluation_results, const ObservedWorldPtr& predicted_world,
                                                          std::vector<mcts::Reward>& rewards,  mcts::EgoCosts& ego_cost) const;

    const std::vector<BehaviorModelPtr>& behavior_hypotheses_;
    mutable std::unordered_map<AgentId, BehaviorModelPtr> behaviors_stored_;

  private:
    std::shared_ptr<MctsStateHypothesis<T>> impl_clone(std::false_type) const {
      return static_cast<const T*>(this)->clone();
    }

    std::shared_ptr<MctsStateHypothesis<T>> impl_clone(std::true_type) const {
      auto worldptr =
          std::dynamic_pointer_cast<ObservedWorld>(observed_world_->Clone());
      return std::make_shared<MctsStateHypothesis<T>>(
          worldptr, is_terminal_state_, num_ego_actions_, prediction_time_span_,
          MctsStateHypothesis<T>::current_agents_hypothesis_, behavior_hypotheses_,
            ego_agent_id_, state_parameters_);
    }

    auto impl_generate_next_state(std::true_type, const EvaluationResults& evaluation_results, const ObservedWorldPtr& predicted_world,
                                                        std::vector<mcts::Reward>& rewards,  mcts::EgoCosts& ego_cost) const {
        rewards.resize(this->get_num_agents(), 0.0f);
        rewards[this->ego_agent_idx] = reward_from_evaluation_results(evaluation_results, state_parameters_);

        ego_cost.resize(1);
        ego_cost[0] = (evaluation_results.collision_drivable_area || evaluation_results.collision_other_agent || evaluation_results.out_of_map) *state_parameters_.COLLISION_COST +
            evaluation_results.goal_reached * state_parameters_.GOAL_COST;
        VLOG_IF_EVERY_N(5, ego_cost[0] != 0.0f || rewards[this->ego_agent_idx] != 0.0, 20) << "Ego reward: " << rewards[this->ego_agent_idx] << ", Ego cost: " << ego_cost[0];

        return std::make_shared<MctsStateHypothesis<T>>(
                    predicted_world, evaluation_results.is_terminal, num_ego_actions_, prediction_time_span_,
                    MctsStateHypothesis<T>::current_agents_hypothesis_, behavior_hypotheses_, 
                    ego_agent_id_, state_parameters_);
    }

    auto impl_generate_next_state(std::false_type, const EvaluationResults& evaluation_results, const ObservedWorldPtr& predicted_world,
                                                        std::vector<mcts::Reward>& rewards,  mcts::EgoCosts& ego_cost) const {
        return static_cast<const T*>(this)->generate_next_state(evaluation_results, predicted_world, rewards, ego_cost);
    }
};

}  // namespace behavior
}  // namespace models
}  // namespace bark

#include "bark_mcts/models/behavior/mcts_state/mcts_state_hypothesis.cpp"

#endif // BARK_MCTS_HYPOTHESIS_STATE_H_