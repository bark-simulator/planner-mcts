// Copyright (c) 2020 Julian Bernhard
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef MODULES_MODELS_BEHAVIOR_BEHAVIOR_UCT_HYPOTHESIS_BASE_HPP_
#define MODULES_MODELS_BEHAVIOR_BEHAVIOR_UCT_HYPOTHESIS_BASE_HPP_

#include <memory>
#include "bark_mcts/models/behavior/behavior_uct_base.hpp"
#include "bark_mcts/models/behavior/mcts_state/mcts_state_hypothesis.hpp"
#include "mcts/hypothesis/hypothesis_belief_tracker.h"

namespace bark {
namespace world {
class ObservedWorld;
}
namespace models {
namespace behavior {

template <class T>
class BehaviorUCTHypothesisBase : public BehaviorUCTBase {
 public:
  explicit BehaviorUCTHypothesisBase(const commons::ParamsPtr& params,
                                const std::vector<BehaviorModelPtr>& behavior_hypothesis);

  virtual ~BehaviorUCTHypothesisBase() {}

  const mcts::HypothesisBeliefTracker& GetBeliefTracker() const { return belief_tracker_;}

  std::vector<BehaviorModelPtr> GetBehaviorHypotheses() const { return behavior_hypotheses_; }

  void UpdateBeliefs(const ObservedWorld& observed_world);

  // Only for (de)serialization purposes
  std::unordered_map<mcts::AgentIdx, std::vector<mcts::Belief>> GetCurrentBeliefs() const { return current_beliefs_; } 
  void SetCurrentBeliefs(const std::unordered_map<mcts::AgentIdx, std::vector<mcts::Belief>>& beliefs) { current_beliefs_ = beliefs; }

 protected:
  void DefineTrueBehaviorsAsHypothesis(const world::ObservedWorld& observed_world);

  // Prediction models (ego and hypothesis)
  std::vector<BehaviorModelPtr> behavior_hypotheses_;
  bool use_true_behaviors_as_hypothesis_;

  // Belief tracking, we must also maintain previous mcts hypothesis state
  mcts::HypothesisBeliefTracker belief_tracker_;
  std::shared_ptr<MctsStateHypothesis<>> last_mcts_hypothesis_state_;

  // Drawing debugging
  std::unordered_map<mcts::AgentIdx, std::vector<mcts::Belief>> current_beliefs_;
};

template<class T>
BehaviorUCTHypothesisBase<T>::BehaviorUCTHypothesisBase(
    const commons::ParamsPtr& params,
    const std::vector<BehaviorModelPtr>& behavior_hypothesis)
    : BehaviorUCTBase(params),
      behavior_hypotheses_(behavior_hypothesis),
      use_true_behaviors_as_hypothesis_(GetParams()->AddChild("BehaviorUctHypothesis")
                                        ->AddChild("PredictionSettings")
                                        ->GetBool("UseTrueBehaviorsAsHypothesis", "When true behaviors out of observed world are used as hypothesis", false)),
      belief_tracker_(mcts_parameters_),
      last_mcts_hypothesis_state_(),
      current_beliefs_() {}

template<class T>
void BehaviorUCTHypothesisBase<T>::DefineTrueBehaviorsAsHypothesis(const world::ObservedWorld& observed_world) {
  // If now hypothesis set specified we take true behaviors as hypothesis
  behavior_hypotheses_.clear();
  std::unordered_map<mcts::AgentIdx, mcts::HypothesisId> fixed_hypothesis_map;
  for (const auto& agent : observed_world.GetOtherAgents()) {
    fixed_hypothesis_map[agent.first] = behavior_hypotheses_.size();
    behavior_hypotheses_.push_back(agent.second->GetBehaviorModel());
  }
  belief_tracker_.update_fixed_hypothesis_set(fixed_hypothesis_map);
}

template<class T>
void BehaviorUCTHypothesisBase<T>::UpdateBeliefs(const ObservedWorld& observed_world) {
    auto world_ptr = std::make_shared<ObservedWorld>(observed_world);
    auto mcts_hypothesis_state_ptr = std::make_shared<MctsStateHypothesis<>>(
                                world_ptr, 
                                false, // terminal
                                0, // num action 
                                0.0, 
                                std::unordered_map<mcts::AgentIdx, mcts::HypothesisId>(), // pass hypothesis reference to states
                                behavior_hypotheses_,
                                observed_world.GetEgoAgentId(),
                                this->mcts_state_parameters_);
    if(!use_true_behaviors_as_hypothesis_) {
      // if this is first call init belief tracker
      if(!last_mcts_hypothesis_state_) {
        belief_tracker_.belief_update(*mcts_hypothesis_state_ptr, *mcts_hypothesis_state_ptr);
      } else {
        belief_tracker_.belief_update(*last_mcts_hypothesis_state_, *mcts_hypothesis_state_ptr);
      }
      last_mcts_hypothesis_state_ = mcts_hypothesis_state_ptr;
      SetCurrentBeliefs(belief_tracker_.get_beliefs());
  }
}

}  // namespace behavior
}  // namespace models
}  // namespace bark

#endif  // MODULES_MODELS_BEHAVIOR_BEHAVIOR_UCT_HYPOTHESIS_BASE_HPP_