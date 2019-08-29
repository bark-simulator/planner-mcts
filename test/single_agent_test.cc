// Copyright (c) 2019 Julian Bernhard
// 
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.
// ========================================================

#include "gtest/gtest.h"
#include "mcts/mcts.h"
#include "src/mcts_state_single_agent.hpp"
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
using modules::geometry::Polygon;
using modules::geometry::Point2d;
using modules::geometry::Pose;
using modules::world::goal_definition::GoalDefinition;

std::mt19937  mcts::RandomGenerator::random_generator_;

TEST(single_agent_mcts_state, execute) {
  // Setup prediction models for ego agent and other agents   
  DefaultParams params;
  DynamicModelPtr dyn_model(new SingleTrackModel());
  BehaviorModelPtr ego_prediction_model(new BehaviorMotionPrimitives(dyn_model, &params));
  float ego_velocity = 5.0, rel_distance = 7.0, velocity_difference=0.0, prediction_time_span=0.2f;
  Input u1(2);  u1 << 0, 0;
  Input u2(2);  u2 << 50, 2; //  < crazy action to drive out of the corridors
  Input u3(2);  u3 << (rel_distance+4)*2/(prediction_time_span*prediction_time_span), 0; //  < action to drive into other agent with a single step
                                                                                        //   (4m vehicle length assumed)
  std::dynamic_pointer_cast<BehaviorMotionPrimitives>(ego_prediction_model)->AddMotionPrimitive(u1);
  std::dynamic_pointer_cast<BehaviorMotionPrimitives>(ego_prediction_model)->AddMotionPrimitive(u2);
  std::dynamic_pointer_cast<BehaviorMotionPrimitives>(ego_prediction_model)->AddMotionPrimitive(u3);
  BehaviorModelPtr others_prediction_model(new BehaviorConstantVelocity(&params));
  PredictionSettings prediction_settings(ego_prediction_model, others_prediction_model);

  // Create an observed world with specific goal definition and the corresponding mcts state
  Polygon polygon(Pose(1, 1, 0), std::vector<Point2d>{Point2d(0, 0), Point2d(0, 2), Point2d(2, 2), Point2d(2, 0), Point2d(0, 0)});
  std::shared_ptr<Polygon> goal_polygon(dynamic_cast<Polygon*>(polygon.translate(Point2d(200,0)))); // < move the goal polygon into the driving corridor in front of the ego vehicle

  auto observed_world = ObservedWorldPtr(make_test_observed_world(1,rel_distance, ego_velocity, velocity_difference, GoalDefinition(*goal_polygon)).Clone());
  observed_world->SetupPrediction(prediction_settings);
  const auto num_ego_actions = std::dynamic_pointer_cast<BehaviorMotionPrimitives>(ego_prediction_model)->GetNumMotionPrimitives();
  MctsStateSingleAgent mcts_state(observed_world, false, num_ego_actions, prediction_time_span);
  
  std::vector<mcts::Reward> rewards;
  auto next_mcts_state = mcts_state.execute(JointAction({0}), rewards);

  // Checking reward of zero if we are neither colliding or at goal
  EXPECT_FALSE(next_mcts_state->is_terminal()); // < make test world is defined in such a way that
                                                // a long driving corridor along x exists
                                                // no collision should occur after one action
  EXPECT_NEAR(rewards[0], 0 , 0.00001);

  // Checking collision with corridor
  next_mcts_state = next_mcts_state->execute(JointAction({1}), rewards);
  EXPECT_TRUE(next_mcts_state->is_terminal()); // < crazy action should lead to a collision with driving corridor
  EXPECT_NEAR(rewards[0], -1000 , 0.00001);

  // Checking collision with other agent ( use initial state again)
  next_mcts_state = mcts_state.execute(JointAction({2}), rewards);
  EXPECT_TRUE(next_mcts_state->is_terminal()); // < action 3 should lead to a collision with other agent
  EXPECT_NEAR(rewards[0], -1000 , 0.00001);


  // Checking goal reached after stepping multiple times: Do multiple steps and expect that goal is reached
  bool reached = false;
  next_mcts_state = mcts_state.execute(JointAction({0}), rewards);
  reached = next_mcts_state->is_terminal();
  for (int i = 0; i < 10000; ++i) {
    if(next_mcts_state->is_terminal()) {
      reached = true;
      break;
    }
    next_mcts_state = next_mcts_state->execute(JointAction({0}), rewards);
  }
  EXPECT_TRUE(reached);
  EXPECT_NEAR(rewards[0], 1 , 0.00001); // < reward should be one when reaching the goal



}



int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();

}