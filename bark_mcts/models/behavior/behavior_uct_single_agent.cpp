// Copyright (c) 2020 Julian Bernhard
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "bark_mcts/models/behavior/behavior_uct_single_agent.hpp"

#include "bark/models/behavior/idm/idm_classic.hpp"
#include "bark/models/behavior/motion_primitives/continuous_actions.hpp"
#include "bark/models/dynamic/single_track.hpp"

namespace bark {
namespace models {
namespace behavior {

using bark::models::dynamic::Input;
using bark::models::dynamic::SingleTrackModel;
using bark::world::prediction::PredictionSettings;

PredictionSettings BehaviorUCTSingleAgent::SetupPredictionSettings(
    const commons::ParamsPtr& params) {
  // Setup prediction models for ego agent and other agents
  DynamicModelPtr dyn_model(new SingleTrackModel(params));
  BehaviorModelPtr ego_prediction_model(
      new BehaviorMPContinuousActions(params));

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
}  // namespace bark
