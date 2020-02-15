// Copyright (c) 2019 fortiss GmbH
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef MODULES_MODELS_BEHAVIOR_UCT_SINGLE_AGENT_MACRO_ACTIONS_HPP_
#define MODULES_MODELS_BEHAVIOR_UCT_SINGLE_AGENT_MACRO_ACTIONS_HPP_

#include <memory>
#include "src/behavior_uct_single_agent_base.hpp"

namespace modules {
namespace models {
namespace behavior {

class BehaviorUCTSingleAgentMacroActions : public BehaviorUCTSingleAgentBase {
 public:
  BehaviorUCTSingleAgentMacroActions(const commons::ParamsPtr& params)
      : BehaviorUCTSingleAgentBase(params) {
    prediction_settings_ = SetupPredictionSettings(params);
  }

  virtual ~BehaviorUCTSingleAgentMacroActions() {}

  virtual modules::world::prediction::PredictionSettings
  SetupPredictionSettings(const commons::ParamsPtr& params);

  virtual std::shared_ptr<BehaviorModel> Clone() const;
};

inline std::shared_ptr<BehaviorModel>
BehaviorUCTSingleAgentMacroActions::Clone() const {
  std::shared_ptr<BehaviorUCTSingleAgentMacroActions> model_ptr =
      std::make_shared<BehaviorUCTSingleAgentMacroActions>(*this);
  return model_ptr;
}

}  // namespace behavior
}  // namespace models
}  // namespace modules

#endif  // MODULES_MODELS_BEHAVIOR_UCT_SINGLE_AGENT_MACRO_ACTIONS_HPP_
