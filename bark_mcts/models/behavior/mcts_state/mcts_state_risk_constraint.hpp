// Copyright (c) 2019 Julian Bernhard
// 
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.
// ========================================================

#ifndef BARK_MCTS_STATE_RISK_CONSTRAINT_H_
#define BARK_MCTS_STATE_RISK_CONSTRAINT_H_


#include "bark_mcts/models/behavior/mcts_state/mcts_state_hypothesis.hpp"

namespace bark {
namespace world {
class ObservedWorld;
typedef std::shared_ptr<ObservedWorld> ObservedWorldPtr;
}  // namespace world
namespace models {
namespace behavior {

using BarkAction = bark::models::behavior::Action;

class MctsStateRiskConstraint : public MctsStateHypothesis<MctsStateRiskConstraint> {
public:
    MctsStateRiskConstraint(const bark::world::ObservedWorldPtr& observed_world,
                       bool is_terminal_state,
                       const mcts::ActionIdx& num_ego_actions,
                       const float& prediction_time_span,
                       const std::unordered_map<mcts::AgentIdx, mcts::HypothesisId>& current_agents_hypothesis,
                       const std::vector<BehaviorModelPtr>& behavior_hypothesis,
                       const BehaviorMotionPrimitivesPtr& ego_behavior_model,
                       const mcts::AgentIdx& ego_agent_id,
                       const StateParameters& state_parameters,
                       const std::unordered_map<mcts::AgentIdx, std::vector<mcts::Belief>>& current_hypothesis_beliefs,
                       const bark::commons::Probability& state_sequence_probability);

   std::shared_ptr<MctsStateRiskConstraint> clone() const;

   bark::commons::Probability calculation_state_transition_probability(
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

   mcts::Cost calculate_collision_cost(const ObservedWorld& to) const {
      return 1.0/10.0f;
    }

    bark::commons::Probability calculate_sequence_probability(const ObservedWorld& to) const {
      return get_state_sequence_probability() * calculation_state_transition_probability(to);
    }

    bark::commons::Probability get_state_sequence_probability() const {
      return state_sequence_probability_;
    }

    typedef BarkAction ActionType; // required for template-mechanism to compile

 private:
  std::shared_ptr<MctsStateRiskConstraint> generate_next_state(const EvaluationResults& evaluation_results, const ObservedWorldPtr& predicted_world) const;

  void calculate_ego_reward_cost(const EvaluationResults& evaluation_results, std::vector<mcts::Reward>& rewards,  mcts::Cost& ego_cost) const;

  const std::unordered_map<mcts::AgentIdx, std::vector<mcts::Belief>>& current_hypothesis_beliefs_;
  const bark::commons::Probability state_sequence_probability_; // Probability to end in this state given the past expanded joint actions
};

}  // namespace behavior
}  // namespace models
}  // namespace bark

#endif // BARK_MCTS_STATE_RISK_CONSTRAINT_H_