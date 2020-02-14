// Copyright (c) 2019 fortiss GmbH
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef MODULES_MODELS_BEHAVIOR_SINGLE_AGENT_MCTS_HPP_
#define MODULES_MODELS_BEHAVIOR_SINGLE_AGENT_MCTS_HPP_

#include "mcts/mcts_parameters.h"

#include <memory>
#include "modules/world/prediction/prediction_settings.hpp"
#include "src/behavior_uct_single_agent_base.hpp"

namespace modules {
namespace models {
namespace behavior {

class BehaviorUCTSingleAgent : public BehaviorUCTSingleAgentBase {
 public:
  BehaviorUCTSingleAgent(const commons::ParamsPtr& params)
      : BehaviorUCTSingleAgentBase(params) {
    prediction_settings_ = SetupPredictionSettings(params);
  }

  virtual ~BehaviorUCTSingleAgent() {}

  virtual modules::world::prediction::PredictionSettings
  SetupPredictionSettings(const commons::ParamsPtr& params);

  virtual std::shared_ptr<BehaviorModel> Clone() const;
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
