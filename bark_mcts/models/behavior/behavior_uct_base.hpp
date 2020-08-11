// Copyright (c) 2020 Julian Bernhard
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef MODULES_MODELS_BEHAVIOR_BEHAVIOR_UCT_BASE_HPP_
#define MODULES_MODELS_BEHAVIOR_BEHAVIOR_UCT_BASE_HPP_

#include <memory>
#include "mcts/mcts_parameters.h"
#include "bark/models/behavior/behavior_model.hpp"
#include "bark/models/behavior/motion_primitives/motion_primitives.hpp"

#include "bark_mcts/models/behavior/mcts_state/mcts_state_base.hpp"

namespace bark {
namespace world {
class ObservedWorld;
}
namespace models {
namespace behavior {

class BehaviorUCTBase : public BehaviorModel {
 public:
  explicit BehaviorUCTBase(const commons::ParamsPtr& params);
  virtual ~BehaviorUCTBase() {}

  BehaviorMotionPrimitivesPtr GetEgoBehavior() const { return ego_behavior_model_; }

 protected:
  BehaviorMotionPrimitivesPtr ego_behavior_model_;

  // PARAMETERS
  const mcts::MctsParameters mcts_parameters_;
  bool dump_tree_;
  double prediction_time_span_;
  StateParameters mcts_state_parameters_;
};


}  // namespace behavior
}  // namespace models
}  // namespace bark

#endif  // MODULES_MODELS_BEHAVIOR_BEHAVIOR_UCT_BASE_HPP_