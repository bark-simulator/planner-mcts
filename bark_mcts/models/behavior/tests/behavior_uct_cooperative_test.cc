// Copyright (c) 2020 Julian Bernhard
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.
#include <chrono>
#include "gtest/gtest.h"
#include "bark_mcts/models/behavior/mcts_state/mcts_state_cooperative.hpp"
#include "bark_mcts/models/behavior/behavior_uct_cooperative.hpp"
#include "bark/models/behavior/motion_primitives/param_config/behav_macro_actions_from_param_server.hpp"
#include "bark_mcts/models/behavior/param_config/mcts_parameters_from_param_server.hpp"
#include "bark_mcts/models/behavior/param_config/mcts_state_parameters_from_param_server.hpp"
#include "bark_mcts/models/behavior/tests/test_helpers.hpp"
#include "bark/commons/params/setter_params.hpp"

#include "bark/world/tests/make_test_world.hpp"
#include "bark/world/tests/make_test_xodr_map.hpp"
#include "bark/models/behavior/motion_primitives/motion_primitives.hpp"
#include "bark/models/behavior/motion_primitives/macro_actions.hpp"
#include "bark/models/behavior/constant_acceleration/constant_acceleration.hpp"
#include "bark/models/dynamic/single_track.hpp"

#include "bark/world/evaluation/evaluator_drivable_area.hpp"
#include "bark/world/evaluation/evaluator_collision_ego_agent.hpp"
#include "bark/world/evaluation/evaluator_goal_reached.hpp"
#include "bark/world/goal_definition/goal_definition_polygon.hpp"
#include "bark/world/goal_definition/goal_definition_state_limits_frenet.hpp"


using namespace bark::models::behavior;
using namespace mcts;
using namespace bark::world::tests;
using bark::world::tests::make_test_observed_world;
using bark::world::prediction::PredictionSettings;
using bark::models::dynamic::SingleTrackModel;
using bark::models::dynamic::Input;
using bark::world::ObservedWorldPtr;
using bark::commons::SetterParams;
using bark::commons::ParamsPtr;

using bark::geometry::Polygon;
using bark::geometry::Point2d;
using bark::geometry::Pose;
using bark::world::goal_definition::GoalDefinitionPtr;
using bark::world::goal_definition::GoalDefinitionPolygon;
using bark::world::goal_definition::GoalDefinitionStateLimitsFrenet;
using bark::models::dynamic::Trajectory;
using bark::world::evaluation::EvaluatorDrivableArea;
using bark::world::evaluation::EvaluatorGoalReached;
using bark::world::evaluation::EvaluatorCollisionEgoAgent;
using bark::world::map::MapInterface;
using bark::world::map::MapInterfacePtr;
using bark::world::objects::Agent;
using bark::world::WorldPtr;
using bark::world::World;
using bark::world::ObservedWorldPtr;
using bark::world::objects::AgentPtr;
using bark::world::opendrive::OpenDriveMapPtr;
using bark::world::tests::MakeXodrMapOneRoadTwoLanes;

TEST(cooperative_mcts_state, execute) {
  // Setup prediction models for ego agent and other agents   
  auto params = std::make_shared<SetterParams>(false);

  params->SetReal("Mcts::State::GoalReward", 200.0);
  params->SetReal("Mcts::State::CollisionReward", -2000.0);
  params->SetReal("Mcts::State::GoalCost", 10.0);
  params->SetReal("Mcts::State::CollisionCost", 300);
  params->SetReal("Mcts::State::CooperationFactor", 0.75);
  params->SetListFloat("AccelerationInputs", {0, 1, 4, -1, -8});

  auto ego_behavior_model = BehaviorMacroActionsFromParamServer(
                                              params);

  // Create an observed world with specific goal definition and the corresponding mcts state
  Polygon polygon(Pose(1, 1, 0), std::vector<Point2d>{Point2d(-4, 4), Point2d(-4, 4), Point2d(4, 4), Point2d(4, -4), Point2d(-4, -4)});
  std::shared_ptr<Polygon> goal_polygon(std::dynamic_pointer_cast<Polygon>(polygon.Translate(Point2d(40, -1.75)))); // < move the goal polygon into the driving corridor in front of the ego vehicle
  auto goal_definition_ptr = std::make_shared<GoalDefinitionPolygon>(*goal_polygon);

  double rel_distance = 10.0f, ego_velocity = 5.0f, velocity_difference = -4.0f, prediction_time_span = 1.0f;
  auto observed_world = std::dynamic_pointer_cast<ObservedWorld>(
        make_test_observed_world(1, rel_distance, ego_velocity, velocity_difference, goal_definition_ptr).Clone());
  observed_world->SetRemoveAgents(true);
  for ( auto& agent : observed_world->GetAgents()) {
    agent.second->SetBehaviorModel(ego_behavior_model);
  }
  auto const_observed_world = std::const_pointer_cast<ObservedWorld>(observed_world);
  const auto num_ego_actions = std::dynamic_pointer_cast<BehaviorMotionPrimitives>(ego_behavior_model)->GetNumMotionPrimitives(const_observed_world);

  // Init hypothesis mcts state
  auto ego_agent_id = observed_world->GetAgents().begin()->first;
  auto front_agent_id = std::next(observed_world->GetAgents().begin())->first;
  auto state_params = MctsStateParametersFromParamServer(params);
  MctsStateCooperative mcts_state(const_observed_world,
                        false,
                       num_ego_actions,
                       prediction_time_span,
                       ego_agent_id,
                       state_params);

  std::vector<mcts::Reward> rewards;
  mcts::EgoCosts cost;
  auto next_mcts_state = mcts_state.execute(JointAction({0, 2}), rewards, cost);

  // Checking reward of zero if we are neither colliding or at goal
  EXPECT_FALSE(next_mcts_state->is_terminal()); // < make test world is defined in such a way that
                                                // a long driving corridor along x exists
                                                // no collision should occur after one action
  EXPECT_NEAR(rewards[0], 0.0f , 0.00001);
  EXPECT_NEAR(cost.at(0), 0,0.001);

  // Clone test
  const auto cloned_state = mcts_state.clone();

  // Checking collision with other agent
  next_mcts_state = mcts_state.execute(JointAction({2, 2}), rewards, cost);
  auto reached = next_mcts_state->is_terminal();
  for (int i = 0; i < 1000; ++i) {
    LOG(INFO) << "---------------\n" << next_mcts_state->sprintf();
    if(next_mcts_state->is_terminal()) {
      reached = true;
      break;
    }
    next_mcts_state = next_mcts_state->execute(JointAction({2, 0}), rewards, cost);
  }
  const auto cf = params->GetReal("Mcts::State::CooperationFactor", "", 1.0f);
  const auto cr = params->GetReal("Mcts::State::CollisionReward", "", 1.0);
  const auto desired_collision_reward = ((1 - cf) * cr + cf * cr) * 0.5;
  EXPECT_TRUE(next_mcts_state->is_terminal()); // < acceleration should lead to a collision with other agent
  EXPECT_NEAR(rewards[0], desired_collision_reward, 0.00001);
  EXPECT_NEAR(cost.at(0), - desired_collision_reward, 0.00001);


  // Checking goal reached: Do multiple steps and expect that goal is reached
  next_mcts_state = mcts_state.execute(JointAction({0, 2}), rewards, cost);
  for (int i = 0; i < 1000; ++i) {
    if(next_mcts_state->is_terminal()) {
      break;
    }
    next_mcts_state = next_mcts_state->execute(JointAction({0, 2}), rewards, cost);
  }
  const auto gr = params->GetReal("Mcts::State::GoalReward", "", 1.0);
  const auto desired_goal_reward =  ( (1 - cf)* gr + cf * 0.0 ) / 2.0;
  EXPECT_NEAR(rewards[0], desired_goal_reward , 0.00001); // < reward should be one when reaching the goal 
  EXPECT_NEAR(cost.at(0), - desired_goal_reward , 0.00001);
}

TEST(behavior_uct_cooperative, no_agent_in_front_accelerate) {
  // Test if uct planner accelerates if there is no agent in front
  auto params = std::make_shared<SetterParams>();
  params->SetBool("BehaviorUctBase::EgoBehavior::BehaviorMPMacroActions::CheckValidityInPlan", false);

  float ego_velocity = 2.0, rel_distance = 7.0, velocity_difference=0.0, prediction_time_span=0.5f;
  Polygon polygon(Pose(1, 1, 0), std::vector<Point2d>{Point2d(-3, 3), Point2d(-3, 3), Point2d(3, 3), Point2d(3, -3), Point2d(-3, -3)});
  std::shared_ptr<Polygon> goal_polygon(std::dynamic_pointer_cast<Polygon>(polygon.Translate(Point2d(150, -1.75)))); // < move the goal polygon into the driving corridor in front of the ego vehicle
  auto goal_definition_ptr = std::make_shared<GoalDefinitionPolygon>(*goal_polygon);
  
  auto observed_world = make_test_observed_world(0,rel_distance, ego_velocity, velocity_difference, goal_definition_ptr);
  observed_world.SetRemoveAgents(true);

  bark::models::behavior::BehaviorUCTCooperative behavior_uct(params);

  Trajectory trajectory = behavior_uct.Plan(prediction_time_span, observed_world);
  auto action = behavior_uct.GetLastAction();
  EXPECT_TRUE(boost::get<bark::models::dynamic::Input>(action)[0]>= 0.0); // << max, available acceleration is action 2
}


TEST(behavior_uct_single_agent, agent_in_front_must_brake) {
  // Test if uct planner brakes when slow agent is directly in front
  auto params = std::make_shared<SetterParams>();
  params->SetReal("Mcts::State::CooperationFactor", 0.2);
  params->SetBool("BehaviorUctBase::EgoBehavior::BehaviorMPMacroActions::CheckValidityInPlan", false);
  params->SetReal("BehaviorUctBase::Mcts::UctStatistic::ProgressiveWidening::Alpha", 0.5);
  params->SetReal("BehaviorUctBase::Mcts::UctStatistic::ProgressiveWidening::K", 4);

  float ego_velocity = 5.0, rel_distance = 2.0, velocity_difference=2.0, prediction_time_span=0.5f;
  Polygon polygon(Pose(1, 1, 0), std::vector<Point2d>{Point2d(-3, 3), Point2d(-3, 3), Point2d(3, 3), Point2d(3, -3), Point2d(-3, -3)});
  std::shared_ptr<Polygon> goal_polygon(std::dynamic_pointer_cast<Polygon>(polygon.Translate(Point2d(150, -1.75)))); // < move the goal polygon into the driving corridor in front of the ego vehicle
  auto goal_definition_ptr = std::make_shared<GoalDefinitionPolygon>(*goal_polygon);
  
  auto observed_world = make_test_observed_world(2,rel_distance, ego_velocity, velocity_difference, goal_definition_ptr);
  observed_world.SetRemoveAgents(true);
  bark::models::behavior::BehaviorUCTCooperative behavior_uct(params);

  Trajectory trajectory = behavior_uct.Plan(prediction_time_span, observed_world);
  auto action = behavior_uct.GetLastAction();
  EXPECT_TRUE(boost::get<bark::models::dynamic::Input>(action)[0] < 0.0f); // some decceleration should occur
}


int main(int argc, char **argv) {
  FLAGS_alsologtostderr = true;
  FLAGS_v = 3;
  ::testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();

}