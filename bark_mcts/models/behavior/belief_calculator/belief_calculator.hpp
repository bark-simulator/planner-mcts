// Copyright (c) 2020 Julian Bernhard
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef MODULES_MODELS_BEHAVIOR_BELIEF_CALCULATOR_BELIEF_CALCULATOR_HPP_
#define MODULES_MODELS_BEHAVIOR_BELIEF_CALCULATOR_BELIEF_CALCULATOR_HPP_

#include <memory>
#include "bark_mcts/models/behavior/mcts_state/mcts_state_hypothesis.hpp"
#include "mcts/hypothesis/hypothesis_belief_tracker.h"

namespace bark {
namespace world {
class ObservedWorld;
}
namespace models {
namespace behavior {

class BeliefCalculator {
  public:
    explicit BeliefCalculator(const commons::ParamsPtr& params,
                                  const std::vector<BehaviorModelPtr>& behavior_hypothesis);

    ~BeliefCalculator() {}

    void BeliefUpdate(const world::ObservedWorld& observed_world);

    std::unordered_map<bark::world::AgentId, std::vector<mcts::Belief>> GetBeliefs() { return belief_tracker_->get_beliefs(); };
    
    std::vector<BehaviorModelPtr> GetBehaviorHypotheses() const { return behavior_hypotheses_; }

    commons::ParamsPtr GetParams() const { return params_; }

    void Reset();

  private:
    std::vector<BehaviorModelPtr> behavior_hypotheses_;
    std::shared_ptr<mcts::HypothesisBeliefTracker> belief_tracker_;
    std::shared_ptr<MctsStateHypothesis<>> last_mcts_hypothesis_state_;
    const mcts::MctsParameters mcts_parameters_;
    const commons::ParamsPtr params_;
};


}  // namespace behavior
}  // namespace models
}  // namespace bark

#endif  // MODULES_MODELS_BEHAVIOR_BELIEF_CALCULATOR_BELIEF_CALCULATOR_HPP_