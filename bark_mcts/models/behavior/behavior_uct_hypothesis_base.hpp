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

 protected:
  void DefineTrueBehaviorsAsHypothesis(const world::ObservedWorld& observed_world);

  // Prediction models (ego and hypothesis)
  std::vector<BehaviorModelPtr> behavior_hypotheses_;
  bool use_true_behaviors_as_hypothesis_;

  // Belief tracking, we must also maintain previous mcts hypothesis state
  mcts::HypothesisBeliefTracker belief_tracker_;
  std::shared_ptr<T> last_mcts_hypothesis_state_;
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
      last_mcts_hypothesis_state_() {}

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


}  // namespace behavior
}  // namespace models
}  // namespace bark

#endif  // MODULES_MODELS_BEHAVIOR_BEHAVIOR_UCT_HYPOTHESIS_BASE_HPP_