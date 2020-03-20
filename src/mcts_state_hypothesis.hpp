// Copyright (c) 2019 Julian Bernhard
// 
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.
// ========================================================

#ifndef BARK_MCTS_HYPOTHESIS_STATE_H_
#define BARK_MCTS_HYPOTHESIS_STATE_H_

// BARK
#include "modules/models/behavior/behavior_model.hpp"
#include "modules/models/behavior/action_store/behavior_action_store.hpp"
#include "modules/models/behavior/hypothesis/behavior_hypothesis.hpp"
#include "modules/models/behavior/motion_primitives/motion_primitives.hpp"
// MCTS Library
#include "mcts/hypothesis/hypothesis_state.h"


namespace modules {
namespace world {
class ObservedWorld;
typedef std::shared_ptr<ObservedWorld> ObservedWorldPtr;
}  // namespace world
namespace models {
namespace behavior {

using BarkAction = modules::models::behavior::Action;

class MctsStateHypothesis : public mcts::HypothesisStateInterface<MctsStateHypothesis> {
public:
    MctsStateHypothesis(const modules::world::ObservedWorldPtr& observed_world,
                       bool is_terminal_state,
                       const mcts::ActionIdx& num_ego_actions,
                       const float& prediction_time_span,
                       const std::unordered_map<mcts::AgentIdx, mcts::HypothesisId>& current_agents_hypothesis,
                       const std::vector<BehaviorHypothesisPtr>& behavior_hypothesis,
                       const BehaviorMotionPrimitivesPtr& ego_behavior_model,
                       const std::vector<mcts::AgentIdx>& other_agent_ids,
                       const mcts::AgentIdx& ego_agent_id);

// General Interfaces MCTS State: todo(@bernhard) move to a generic base class
    std::shared_ptr<MctsStateHypothesis> execute(const mcts::JointAction &joint_action,
                                            std::vector<mcts::Reward>& rewards,
                                            mcts::Cost& ego_cost) const;

    std::shared_ptr<MctsStateHypothesis> clone() const;

    mcts::ActionIdx get_num_actions(mcts::AgentIdx agent_idx) const { return num_ego_actions_; }

    bool is_terminal() const { return is_terminal_state_; };

    const std::vector<mcts::AgentIdx> get_other_agent_idx() const;

    const mcts::AgentIdx get_ego_agent_idx() const;

    std::string sprintf() const { return std::string(); };

    // Hypothesis State Interfaces
    mcts::ActionIdx plan_action_current_hypothesis(const mcts::AgentIdx& agent_idx) const;

    template<typename ActionType = modules::models::behavior::Action>
    mcts::Probability get_probability(const mcts::HypothesisId& hypothesis, const mcts::AgentIdx& agent_idx, 
                const ActionType& action) const; 

    template<typename ActionType = modules::models::behavior::Action>
    ActionType get_last_action(const mcts::AgentIdx& agent_idx) const;

    mcts::Probability get_prior(const mcts::HypothesisId& hypothesis, const mcts::AgentIdx& agent_idx) const;

    mcts::HypothesisId get_num_hypothesis(const mcts::AgentIdx& agent_idx) const;

    typedef BarkAction ActionType; // required for template-mechanism to compile

 private:
  const std::shared_ptr<const modules::world::ObservedWorld> observed_world_;
  const bool is_terminal_state_;
  const mcts::ActionIdx num_ego_actions_;
  const float prediction_time_span_;
  const std::vector<mcts::AgentIdx> other_agent_ids_;
  const mcts::AgentIdx ego_agent_id_;

  // ---------------- Hypothesis specific ----------------------
  // available hypothesis and ego model can be shared across all states
  const std::vector<BehaviorHypothesisPtr>& behavior_hypothesis_;
  const BehaviorMotionPrimitivesPtr& ego_behavior_model_;
  mutable std::unordered_map<AgentId, BehaviorModelPtr> behaviors_stored_;

};

}  // namespace behavior
}  // namespace models
}  // namespace modules

#endif // BARK_MCTS_HYPOTHESIS_STATE_H_