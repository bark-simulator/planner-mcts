// Copyright (c) 2019 Julian Bernhard
// 
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.
// ========================================================

#include "gtest/gtest.h"
#include "mcts/mcts.h"
#include "src/single_agent_mcts_state.hpp"
#include "modules/commons/params/default_params.hpp"
#include "modules/models/tests/make_test_world.hpp"
#include "modules/models/behavior/motion_primitives/motion_primitives.hpp"
#include "modules/models/behavior/constant_velocity/constant_velocity.hpp"
#include "modules/models/dynamic/single_track.hpp"
#include "modules/commons/params/default_params.hpp"

using namespace modules::models::behavior;
using namespace mcts;
using modules::models::tests::make_test_observed_world;
using modules::world::prediction::PredictionSettings;
using modules::models::dynamic::SingleTrackModel;
using modules::models::dynamic::Input;
using modules::world::ObservedWorldPtr;
using modules::commons::DefaultParams;

std::mt19937  mcts::RandomGenerator::random_generator_;

TEST(single_agent_mcts_state, execute) {
  // Setup prediction models for ego agent and other agents   
  DefaultParams params;
  DynamicModelPtr dyn_model(new SingleTrackModel());
  BehaviorModelPtr ego_prediction_model(new BehaviorMotionPrimitives(dyn_model, &params));
  Input u(2);  u << 0, 0;
  BehaviorMotionPrimitives::MotionIdx idx = std::dynamic_pointer_cast<BehaviorMotionPrimitives>(ego_prediction_model)->AddMotionPrimitive(u);
  BehaviorModelPtr others_prediction_model(new BehaviorConstantVelocity(&params));
  PredictionSettings prediction_settings(ego_prediction_model, others_prediction_model);

  float ego_velocity = 5.0, rel_distance = 7.0, velocity_difference=0.0;
  auto observed_world = ObservedWorldPtr(make_test_observed_world(1,rel_distance, ego_velocity, velocity_difference).Clone());
  observed_world->SetupPrediction(prediction_settings);
  const auto num_ego_actions = std::dynamic_pointer_cast<BehaviorMotionPrimitives>(ego_prediction_model)->GetNumMotionPrimitives();
  MctsStateSingleAgent mcts_state(observed_world, false, num_ego_actions, 0.2f);
  
  std::vector<mcts::Reward> rewards;
  auto next_mcts_state = mcts_state.execute(JointAction({0}), rewards);
  EXPECT_TRUE(next_mcts_state->is_terminal());
  EXPECT_NEAR(rewards[0], -1000 , 0.00001);
}



int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();

}