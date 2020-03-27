// Copyright (c) 2019 fortiss GmbH
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "modules/models/behavior/behavior_uct_single_agent.hpp"

#include "modules/models/behavior/idm/idm_classic.hpp"
#include "modules/models/behavior/motion_primitives/continuous_actions.hpp"
#include "modules/models/dynamic/single_track.hpp"

namespace modules {
namespace models {
namespace behavior {

using modules::models::dynamic::Input;
using modules::models::dynamic::SingleTrackModel;
using modules::world::prediction::PredictionSettings;

PredictionSettings BehaviorUCTSingleAgent::SetupPredictionSettings(
    const commons::ParamsPtr& params) {
  // Setup prediction models for ego agent and other agents
  DynamicModelPtr dyn_model(new SingleTrackModel(params));
  BehaviorModelPtr ego_prediction_model(
      new BehaviorMPContinuousActions(dyn_model, params));

  auto input_list =
      params->GetListListFloat("BehaviorUctSingleAgent::MotionPrimitiveInputs",
                               "A list of pairs with acceleration and steering",
                               {{0, 0}, {5, 0}, {0, -1}, {0, 1}, {-3, 0}});
  for (const auto& input : input_list) {
    BARK_EXPECT_TRUE(input.size() == 2);
    Input u(2);
    u << input[0], input[1];
    BehaviorMotionPrimitives::MotionIdx idx =
        std::dynamic_pointer_cast<BehaviorMPContinuousActions>(
            ego_prediction_model)
            ->AddMotionPrimitive(u);
  }

  BehaviorModelPtr others_prediction_model(new BehaviorIDMClassic(params));
  return PredictionSettings(ego_prediction_model, others_prediction_model);
}

}  // namespace behavior
}  // namespace models
}  // namespace modules
