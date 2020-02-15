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

using modules::models::behavior::primitives::Primitive;
using modules::models::behavior::primitives::PrimitiveConstAcceleration;
using modules::models::behavior::primitives::PrimitiveChangeToLeft;
using modules::models::behavior::primitives::PrimitiveChangeToRight;
using modules::models::dynamic::Input;
using modules::models::dynamic::SingleTrackModel;
using modules::world::prediction::PredictionSettings;

PredictionSettings BehaviorUCTSingleAgentMacroActions::SetupPredictionSettings(
    const commons::ParamsPtr& params) {
  // Setup prediction models for ego agent and other agents
  DynamicModelPtr dyn_model(new SingleTrackModel(params));
  BehaviorModelPtr ego_prediction_model(
      new BehaviorMPMacroActions(dyn_model, params));

  std::vector<std::shared_ptr<Primitive>> prim_vec;

  //! TODO: Move Parameters to parm server
  float cte = 0.1; // cross track error
  std::vector<float> acc_vec{0, 1, -1};

  for (auto& acc : acc_vec) {
    auto primitive =
        std::make_shared<PrimitiveConstAcceleration>(params, dyn_model, acc, cte);
    prim_vec.push_back(primitive);
  }

  auto primitive_left =
      std::make_shared<PrimitiveChangeToLeft>(params, dyn_model, cte);
  prim_vec.push_back(primitive_left);

  auto primitive_right =
      std::make_shared<PrimitiveChangeToRight>(params, dyn_model, cte);
  prim_vec.push_back(primitive_right);

  for (auto& p : prim_vec) {
    auto idx =
        std::dynamic_pointer_cast<BehaviorMPMacroActions>(ego_prediction_model)
            ->AddMotionPrimitive(p);
  }
  BehaviorModelPtr others_prediction_model(new BehaviorIDMClassic(params));
  return PredictionSettings(ego_prediction_model, others_prediction_model);
}

}  // namespace behavior
}  // namespace models
}  // namespace modules
