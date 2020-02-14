// Copyright (c) 2019 fortiss GmbH
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef MODULES_MODELS_BEHAVIOR_SINGLE_AGENT_MCTS_HPP_
#define MODULES_MODELS_BEHAVIOR_SINGLE_AGENT_MCTS_HPP_

#include "mcts/mcts_parameters.h"

#include <memory>
#include "modules/models/behavior/behavior_model.hpp"
#include "modules/world/prediction/prediction_settings.hpp"

namespace modules {
namespace world {
class ObservedWorld;
}
namespace models {
namespace behavior {

class BehaviorUCTSingleAgent : public BehaviorModel {
 public:
  explicit BehaviorUCTSingleAgent(const commons::ParamsPtr& params);

  virtual Trajectory Plan(float delta_time,
                          const world::ObservedWorld& observed_world);

  modules::world::prediction::PredictionSettings SetupPredictionSettings(
      const commons::ParamsPtr& params);

  virtual ~BehaviorUCTSingleAgent() {}

  virtual std::shared_ptr<BehaviorModel> Clone() const;

 private:
  modules::world::prediction::PredictionSettings prediction_settings_;
  bool dump_tree_;

  // MCTS PARAMETERS
  mcts::MctsParameters mcts_parameters_;
};

inline std::shared_ptr<BehaviorModel> BehaviorUCTSingleAgent::Clone() const {
  std::shared_ptr<BehaviorUCTSingleAgent> model_ptr =
      std::make_shared<BehaviorUCTSingleAgent>(*this);
  return model_ptr;
}

}  // namespace behavior
}  // namespace models
}  // namespace modules

#endif  // MODULES_MODELS_BEHAVIOR_SINGLE_AGENT_MCTS_HPP_
