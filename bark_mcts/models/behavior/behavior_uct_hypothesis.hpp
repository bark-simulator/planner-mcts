// Copyright (c) 2020 Julian Bernhard
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef MODULES_MODELS_BEHAVIOR_BEHAVIOR_UCT_HYPOTHESIS_HPP_
#define MODULES_MODELS_BEHAVIOR_BEHAVIOR_UCT_HYPOTHESIS_HPP_

#include <memory>
#include "mcts/mcts_parameters.h"
#include "bark_mcts/models/behavior/mcts_state/mcts_state_hypothesis.hpp"
#include "mcts/hypothesis/hypothesis_belief_tracker.h"

#include "bark/models/behavior/behavior_model.hpp"
#include "bark/world/prediction/prediction_settings.hpp"

namespace bark {
namespace world {
class ObservedWorld;
}
namespace models {
namespace behavior {

class BehaviorUCTHypothesis : public BehaviorModel {
 public:
  explicit BehaviorUCTHypothesis(const commons::ParamsPtr& params,
                                const std::vector<BehaviorModelPtr>& behavior_hypothesis);

  virtual ~BehaviorUCTHypothesis() {}

  virtual Trajectory Plan(float delta_time,
                          const world::ObservedWorld& observed_world);

  virtual std::shared_ptr<BehaviorModel> Clone() const;

  const mcts::HypothesisBeliefTracker& GetBeliefTracker() const { return belief_tracker_;}

  std::vector<BehaviorModelPtr> GetBehaviorHypotheses() const { return behavior_hypotheses_; }

  BehaviorMotionPrimitivesPtr GetEgoBehavior() const { return ego_behavior_model_; }

 protected:
  void DefineTrueBehaviorsAsHypothesis(const world::ObservedWorld& observed_world);

  // Prediction models (ego and hypothesis)
  std::vector<BehaviorModelPtr> behavior_hypotheses_;
  BehaviorMotionPrimitivesPtr ego_behavior_model_;

  // PARAMETERS
  mcts::MctsParameters mcts_parameters_;
  bool dump_tree_;
  double prediction_time_span_;
  bool use_true_behaviors_as_hypothesis_;
  StateParameters mcts_state_parameters_;

  // Belief tracking, we must also maintain previous mcts hypothesis state
  mcts::HypothesisBeliefTracker belief_tracker_;
  std::shared_ptr<MctsStateHypothesis> last_mcts_hypothesis_state_;
};

inline std::shared_ptr<BehaviorModel> BehaviorUCTHypothesis::Clone() const {
  std::shared_ptr<BehaviorUCTHypothesis> model_ptr =
      std::make_shared<BehaviorUCTHypothesis>(*this);
  return model_ptr;
}

}  // namespace behavior
}  // namespace models
}  // namespace bark

#endif  // MODULES_MODELS_BEHAVIOR_BEHAVIOR_UCT_HYPOTHESIS_HPP_