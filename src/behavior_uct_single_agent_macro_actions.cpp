// Copyright (c) 2019 fortiss GmbH
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "src/behavior_uct_single_agent_macro_actions.hpp"

#include "modules/models/behavior/idm/idm_classic.hpp"
#include "modules/models/behavior/motion_primitives/macro_actions.hpp"
#include "modules/models/dynamic/single_track.hpp"

namespace modules {
namespace models {
namespace behavior {

using modules::models::dynamic::Input;
using modules::models::dynamic::SingleTrackModel;
using modules::world::prediction::PredictionSettings;
using modules::models::behavior::primitives::PrimitiveConstAcceleration;

PredictionSettings BehaviorUCTSingleAgentMacroActions::SetupPredictionSettings(
    const commons::ParamsPtr& params) {
  // Setup prediction models for ego agent and other agents
  DynamicModelPtr dyn_model(new SingleTrackModel(params));
  BehaviorModelPtr ego_prediction_model(
      new BehaviorMPMacroActions(dyn_model, params));

  auto primitive =
      std::make_shared<PrimitiveConstAcceleration>(params, dyn_model, 0, 0.1);
  auto idx =
      std::dynamic_pointer_cast<BehaviorMPMacroActions>(ego_prediction_model)
          ->AddMotionPrimitive(primitive);

  BehaviorModelPtr others_prediction_model(new BehaviorIDMClassic(params));
  return PredictionSettings(ego_prediction_model, others_prediction_model);
}

}  // namespace behavior
}  // namespace models
}  // namespace modules
