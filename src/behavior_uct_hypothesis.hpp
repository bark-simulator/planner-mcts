// Copyright (c) 2019 fortiss GmbH
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef MODULES_MODELS_BEHAVIOR_BEHAVIOR_UCT_HYPOTHESIS_HPP_
#define MODULES_MODELS_BEHAVIOR_BEHAVIOR_UCT_HYPOTHESIS_HPP_

#include <memory>
#include "mcts/mcts_parameters.h"
#include "mcts/hypothesis/hypothesis_belief_tracker.h"

#include "modules/models/behavior/behavior_model.hpp"
#include "modules/world/prediction/prediction_settings.hpp"

namespace modules {
namespace world {
class ObservedWorld;
}
namespace models {
namespace behavior {

class BehaviorUCTHypothesis : public BehaviorModel {
 public:
  explicit BehaviorUCTHypothesis(const commons::ParamsPtr& params);

  virtual ~BehaviorUCTHypothesis() {}

  virtual Trajectory Plan(float delta_time,
                          const world::ObservedWorld& observed_world);

  virtual std::shared_ptr<BehaviorModel> Clone() const = 0;


 protected:
   std::vector<AgentIdx> get_agent_id_map (const world::ObservedWorld &observed_world) const;
   
  // Prediction models (ego and hypothesis)
  std::vector<BehaviorHypothesisPtr>> behavior_hypothesis_;
  BehaviorMotionPrimitivesPtr ego_behavior_model_;

  // PARAMETERS
  mcts::MctsParameters mcts_parameters_;
  bool dump_tree_;
  double prediction_time_span_;

  // Belief tracking, we must also maintain previous mcts hypothesis state
  HypothesisBeliefTracker belief_tracker_;
  std::shared_ptr<MctsStateHypothesis> last_mcts_hypothesis_state_;
};