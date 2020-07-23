// Copyright (c) 2020 Julian Bernhard
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "bark_mcts/models/behavior/behavior_uct_single_agent_macro_actions.hpp"
#include "bark/models/behavior/motion_primitives/param_config/behav_macro_actions_from_param_server.hpp"

#include "bark/models/behavior/idm/idm_classic.hpp"
#include "bark/models/behavior/motion_primitives/macro_actions.hpp"
#include "bark/models/dynamic/single_track.hpp"

namespace bark {
namespace models {
namespace behavior {

using bark::models::behavior::primitives::Primitive;
using bark::models::dynamic::Input;
using bark::models::dynamic::SingleTrackModel;
using bark::world::prediction::PredictionSettings;

PredictionSettings BehaviorUCTSingleAgentMacroActions::SetupPredictionSettings(
    const commons::ParamsPtr& params) {
  // Setup prediction models for ego agent and other agents
  auto prediction_params_ego = params->AddChild("EgoVehicle");
  DynamicModelPtr dyn_model(new SingleTrackModel(prediction_params_ego));
  BehaviorModelPtr ego_prediction_model(models::behavior::
          BehaviorMacroActionsFromParamServer(GetParams()
            ->AddChild("BehaviorUctHypothesis")->AddChild("EgoBehavior")));
      
  auto prediction_params_other = params->AddChild("OtherVehicles");
  BehaviorModelPtr others_prediction_model(
      new BehaviorIDMClassic(prediction_params_other));
  return PredictionSettings(ego_prediction_model, others_prediction_model);
}


}  // namespace behavior
}  // namespace models
}  // namespace bark
