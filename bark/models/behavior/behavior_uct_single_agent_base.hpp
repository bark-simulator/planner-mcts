// Copyright (c) 2019 fortiss GmbH
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef MODULES_MODELS_BEHAVIOR_SINGLE_AGENT_MCTS_BASE_HPP_
#define MODULES_MODELS_BEHAVIOR_SINGLE_AGENT_MCTS_BASE_HPP_

#include <memory>
#include "mcts/mcts_parameters.h"

#include "bark/models/behavior/behavior_model.hpp"
#include "bark/world/prediction/prediction_settings.hpp"

namespace bark {
namespace world {
class ObservedWorld;
}
namespace models {
namespace behavior {

class BehaviorUCTSingleAgentBase : public BehaviorModel {
 public:
  explicit BehaviorUCTSingleAgentBase(const commons::ParamsPtr& params);

  virtual ~BehaviorUCTSingleAgentBase() {}

  virtual Trajectory Plan(float delta_time,
                          const world::ObservedWorld& observed_world);

  virtual bark::world::prediction::PredictionSettings
  SetupPredictionSettings(const commons::ParamsPtr& params) = 0;

  virtual std::shared_ptr<BehaviorModel> Clone() const = 0;

 protected:
  bark::world::prediction::PredictionSettings prediction_settings_;
  bool dump_tree_;

  // MCTS PARAMETERS
  mcts::MctsParameters mcts_parameters_;
};

}  // namespace behavior
}  // namespace models
}  // namespace bark

#endif  // MODULES_MODELS_BEHAVIOR_SINGLE_AGENT_MCTS_BASE_HPP_
