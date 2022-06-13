// Copyright (c) 2020 Julian Bernhard
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "bark/models/behavior/behavior_uct_single_agent_macro_actions.hpp"

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

using bark::models::behavior::primitives::Primitive;
using bark::models::behavior::primitives::PrimitiveConstAccChangeToLeft;
using bark::models::behavior::primitives::PrimitiveConstAccChangeToRight;
using bark::models::behavior::primitives::PrimitiveConstAccStayLane;
using bark::models::behavior::primitives::PrimitiveGapKeeping;
using bark::models::dynamic::Input;
using bark::models::dynamic::SingleTrackModel;
using bark::world::prediction::PredictionSettings;

PredictionSettings BehaviorUCTSingleAgentMacroActions::SetupPredictionSettings(
    const commons::ParamsPtr& params) {
  // Setup prediction models for ego agent and other agents
  auto prediction_params_ego = params->AddChild("EgoVehicle");
  DynamicModelPtr dyn_model(new SingleTrackModel(prediction_params_ego));
  BehaviorModelPtr ego_prediction_model(
      new BehaviorMPMacroActions(dyn_model, prediction_params_ego));

  float cte = prediction_params_ego->GetReal("CrossTrackError",
                              "Parameter for lat control", 1);
  std::vector<float> acc_vec = prediction_params_ego->GetListFloat("AccelerationInputs",
                           "A list of acceleration ", {0, 1, 4, -1, -6});

  std::vector<std::shared_ptr<Primitive>> prim_vec;

  for (auto& acc : acc_vec) {
    auto primitive = std::make_shared<PrimitiveConstAccStayLane>(
        prediction_params_ego, dyn_model, acc, cte);
    prim_vec.push_back(primitive);
  }

  auto primitive_left = std::make_shared<PrimitiveConstAccChangeToLeft>(
      prediction_params_ego, dyn_model, cte);
  prim_vec.push_back(primitive_left);

  auto primitive_right = std::make_shared<PrimitiveConstAccChangeToRight>(
      prediction_params_ego, dyn_model, cte);
  prim_vec.push_back(primitive_right);

  auto primitive_gap_keeping = std::make_shared<PrimitiveGapKeeping>(
      prediction_params_ego, dyn_model);
  prim_vec.push_back(primitive_gap_keeping);

  for (auto& p : prim_vec) {
    auto idx =
        std::dynamic_pointer_cast<BehaviorMPMacroActions>(ego_prediction_model)
            ->AddMotionPrimitive(p);
  }
  auto prediction_params_other = params->AddChild("OtherVehicles");
  BehaviorModelPtr others_prediction_model(
      new BehaviorIDMClassic(prediction_params_other));
  return PredictionSettings(ego_prediction_model, others_prediction_model);
}


}  // namespace behavior
}  // namespace models
}  // namespace bark
