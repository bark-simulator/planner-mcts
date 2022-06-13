// Copyright (c) 2020 Julian Bernhard
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "gtest/gtest.h"
#include "mcts/mcts.h"
#include "bark/models/behavior/mcts_state/mcts_state_single_agent.hpp"
#include "bark/models/behavior/behavior_uct_single_agent_macro_actions.hpp"
#include "bark/commons/params/default_params.hpp"
#include "bark/world/tests/make_test_world.hpp"
#include "bark/models/behavior/motion_primitives/motion_primitives.hpp"
#include "bark/models/behavior/motion_primitives/macro_actions.hpp"
#include "bark/models/behavior/constant_acceleration/constant_acceleration.hpp"
#include "bark/models/dynamic/single_track.hpp"
#include "bark/commons/params/default_params.hpp"
#include "bark/commons/params/setter_params.hpp"
#include "bark/world/evaluation/evaluator_drivable_area.hpp"
#include "bark/world/evaluation/evaluator_collision_ego_agent.hpp"
#include "bark/world/evaluation/evaluator_goal_reached.hpp"
#include "bark/world/goal_definition/goal_definition_polygon.hpp"
#include "bark/world/goal_definition/goal_definition_state_limits.hpp"


using namespace bark::models::behavior;
using namespace mcts;
using bark::world::tests::make_test_observed_world;
using bark::world::tests::make_test_world;
using bark::world::prediction::PredictionSettings;
using bark::models::dynamic::SingleTrackModel;
using bark::models::dynamic::Input;
using bark::world::ObservedWorldPtr;
using bark::commons::DefaultParams;
using bark::commons::SetterParams;
using bark::geometry::Polygon;
using bark::geometry::Point2d;
using bark::geometry::Pose;
using bark::world::goal_definition::GoalDefinitionPtr;
using bark::world::goal_definition::GoalDefinitionPolygon;
using bark::world::goal_definition::GoalDefinitionStateLimits;
using bark::models::dynamic::Trajectory;
using bark::world::evaluation::EvaluatorDrivableArea;
using bark::world::evaluation::EvaluatorGoalReached;
using bark::world::evaluation::EvaluatorCollisionEgoAgent;


TEST(behavior_uct_single_agent_macro_actions, no_agent_in_front_accelerate) {
  // Test if uct planner accelerates if there is no agent in front
  auto params = std::make_shared<SetterParams>(true);
  params->SetInt("BehaviorUctSingleAgent::Mcts::MaxNumIterations", 10000);
  params->SetInt("BehaviorUctSingleAgent::Mcts::MaxSearchTime", 20000);
  params->SetInt("BehaviorUctSingleAgent::Mcts::RandomSeed", 1000);
  params->SetBool("BehaviorUctSingleAgent::DumpTree", true);
  params->SetListListFloat("BehaviorUctSingleAgent::MotionPrimitiveInputs", {{0,0}, {5,0}, {0,-1}, {0, 1}, {-3,0}}); 
  params->SetReal("BehaviorUctSingleAgent::Mcts::DiscountFactor", 0.95);
  params->SetReal("BehaviorUctSingleAgent::Mcts::UctStatistic::ExplorationConstant", 0.7);
  params->SetInt("BehaviorUctSingleAgent::Mcts::RandomHeuristic::MaxSearchTime", 10000);
  params->SetInt("BehaviorUctSingleAgent::Mcts::RandomHeuristic::MaxNumIterations", 10);
  params->SetReal("BehaviorUctSingleAgent::Mcts::UctStatistic::ReturnLowerBound", -1000);
  params->SetReal("BehaviorUctSingleAgent::Mcts::UctStatistic::ReturnUpperBound", 100);

  float ego_velocity = 2.0, rel_distance = 7.0, velocity_difference=0.0, prediction_time_span=0.5f;
  Polygon polygon(Pose(1, 1, 0), std::vector<Point2d>{Point2d(-5, -5), Point2d(-5, 5), Point2d(5, 5), Point2d(5, -5), Point2d(-5, -5)});
  std::shared_ptr<Polygon> goal_polygon(std::dynamic_pointer_cast<Polygon>(polygon.Translate(Point2d(20,0)))); // < move the goal polygon into the driving corridor in front of the ego vehicle
  auto goal_definition_ptr = std::make_shared<GoalDefinitionPolygon>(*goal_polygon);
  
  auto observed_world = make_test_observed_world(0,rel_distance, ego_velocity, velocity_difference, goal_definition_ptr);

  BehaviorUCTSingleAgentMacroActions behavior_uct(params);

  Trajectory trajectory = behavior_uct.Plan(prediction_time_span, observed_world);
}

TEST(behavior_uct_single_agent, agent_in_front_must_brake) {
  // Test if uct planner brakes when slow agent is directly in front
  auto params = std::make_shared<SetterParams>(true);
  params->SetInt("BehaviorUctSingleAgent::Mcts::MaxNumIterations", 1000);
  params->SetInt("BehaviorUctSingleAgent::Mcts::MaxSearchTime", 20000);
  params->SetInt("BehaviorUctSingleAgent::Mcts::RandomSeed", 1000);
  params->SetBool("BehaviorUctSingleAgent::DumpTree", true);
  params->SetReal("BehaviorUctSingleAgent::Mcts::DiscountFactor", 0.9);
  params->SetReal("BehaviorUctSingleAgent::Mcts::UctStatistic::ExplorationConstant", 0.7);
  params->SetInt("BehaviorUctSingleAgent::Mcts::RandomHeuristic::MaxSearchTime", 10);
  params->SetInt("BehaviorUctSingleAgent::Mcts::RandomHeuristic::MaxNumIterations", 100);
  params->SetReal("BehaviorUctSingleAgent::Mcts::UctStatistic::ReturnLowerBound", -1000);
  params->SetReal("BehaviorUctSingleAgent::Mcts::UctStatistic::ReturnUpperBound", 100);

  float ego_velocity = 5.0, rel_distance = 7.0, velocity_difference=2.0, prediction_time_span=0.2f;
  Polygon polygon(Pose(1, 1, 0), std::vector<Point2d>{Point2d(-5, -5), Point2d(-5, 5), Point2d(5, 5), Point2d(5, -5), Point2d(-5, -5)});
  std::shared_ptr<Polygon> goal_polygon(std::dynamic_pointer_cast<Polygon>(polygon.Translate(Point2d(100,0)))); // < move the goal polygon into the driving corridor in front of the ego vehicle
  auto goal_definition_ptr = std::make_shared<GoalDefinitionPolygon>(*goal_polygon);
  
  auto observed_world = make_test_observed_world(1,rel_distance, ego_velocity, velocity_difference, goal_definition_ptr);

  BehaviorUCTSingleAgentMacroActions behavior_uct(params);

  Trajectory trajectory = behavior_uct.Plan(prediction_time_span, observed_world);
}

TEST(behavior_uct_single_agent, agent_in_front_reach_goal) {
  // Test if the planner reaches the goal at some point when agent is slower and in front
  auto params = std::make_shared<SetterParams>();
  params->SetInt("BehaviorUctSingleAgent::Mcts::MaxNumIterations", 1000);
  params->SetInt("BehaviorUctSingleAgent::Mcts::MaxSearchTime", 2000);
  params->SetInt("BehaviorUctSingleAgent::Mcts::RandomSeed", 1000);
  params->SetBool("BehaviorUctSingleAgent::DumpTree", true);
  params->SetReal("BehaviorUctSingleAgent::Mcts::DiscountFactor", 0.9);
  params->SetReal("BehaviorUctSingleAgent::Mcts::UctStatistic::ExplorationConstant", 0.7);
  params->SetInt("BehaviorUctSingleAgent::Mcts::RandomHeuristic::MaxSearchTime",1);
  params->SetInt("BehaviorUctSingleAgent::Mcts::RandomHeuristic::MaxNumIterations", 100);
  params->SetReal("BehaviorUctSingleAgent::Mcts::UctStatistic::ReturnLowerBound", -1000);
  params->SetReal("BehaviorUctSingleAgent::Mcts::UctStatistic::ReturnUpperBound", 100);

  float ego_velocity = 5.0, rel_distance = 2.0, velocity_difference=2.0, prediction_time_span=0.2f;
  Polygon polygon(Pose(1, 1, 0), std::vector<Point2d>{Point2d(-5, -5), Point2d(-5, 5), Point2d(5, 5), Point2d(5, -5), Point2d(-5, -5)});
  std::shared_ptr<Polygon> goal_polygon(std::dynamic_pointer_cast<Polygon>(polygon.Translate(Point2d(10,0)))); // < move the goal polygon into the driving corridor in front of the ego vehicle
  auto goal_definition_ptr = std::make_shared<GoalDefinitionPolygon>(*goal_polygon);
  
  auto world = make_test_world(1,rel_distance, ego_velocity, velocity_difference, goal_definition_ptr);

  BehaviorModelPtr behavior_uct(new BehaviorUCTSingleAgentMacroActions(params));
  world->GetAgents().begin()->second->SetBehaviorModel(behavior_uct);

  auto evaluator_drivable_area = EvaluatorDrivableArea();
  auto evaluator_collision_ego = EvaluatorCollisionEgoAgent(world->GetAgents().begin()->second->GetAgentId());
        

  bool goal_reached = false;
  for(int i =0; i<100; ++i) {
    world->Step(prediction_time_span);
    bool outside_drivable_area = boost::get<bool>(evaluator_drivable_area.Evaluate(*world));
    bool collision_ego = boost::get<bool>(evaluator_collision_ego.Evaluate(*world));
    EXPECT_FALSE(outside_drivable_area);
    EXPECT_FALSE(collision_ego);
    if(world->GetAgents().begin()->second->AtGoal()) {
      goal_reached = true;
      break;
    }
  }
  EXPECT_TRUE(goal_reached);

}

TEST(behavior_uct_single_agent, change_lane) {
  // Test if the planner reaches the goal at some point when agent is slower and in front
  auto params = std::make_shared<SetterParams>();
  params->SetInt("BehaviorUctSingleAgent::Mcts::MaxNumIterations", 400);
  params->SetInt("BehaviorUctSingleAgent::Mcts::MaxSearchTime", 4000);
  params->SetInt("BehaviorUctSingleAgent::Mcts::RandomSeed", 1000);
  params->SetBool("BehaviorUctSingleAgent::DumpTree", true);
  params->SetListListFloat("BehaviorUctSingleAgent::MotionPrimitiveInputs", {{0,0}, {1,0}, {0,-0.27}, {0, 0.27}, {0,-0.17}, {0, 0.17}, {-1,0}}); 
  params->SetReal("BehaviorUctSingleAgent::Mcts::DiscountFactor", 0.9);
  params->SetReal("BehaviorUctSingleAgent::Mcts::UctStatistic::ExplorationConstant", 0.7);
  params->SetInt("BehaviorUctSingleAgent::Mcts::RandomHeuristic::MaxSearchTime", 20000);
  params->SetInt("BehaviorUctSingleAgent::Mcts::RandomHeuristic::MaxNumIterations", 10);
  params->SetReal("BehaviorUctSingleAgent::Mcts::UctStatistic::ReturnLowerBound", -1000);
  params->SetReal("BehaviorUctSingleAgent::Mcts::UctStatistic::ReturnUpperBound", 100);


  float ego_velocity = 5.0, rel_distance = 2.0, velocity_difference=2.0, prediction_time_span=0.2f;
  Polygon polygon(Pose(0, 0, 0), std::vector<Point2d>{Point2d(0, -1), Point2d(0, 1), Point2d(20, 1), Point2d(20, -1), Point2d(0, -1)});
  std::shared_ptr<Polygon> goal_polygon(std::dynamic_pointer_cast<Polygon>(polygon.Translate(Point2d(30, -0.5)))); // < move the goal polygon into the driving corridor to the side of the ego vehicle
  auto goal_definition_ptr = std::make_shared<GoalDefinitionStateLimits>(*goal_polygon, std::make_pair<float, float>(-0.2f, 0.2f));
  
  auto world = make_test_world(0,rel_distance, ego_velocity, velocity_difference, goal_definition_ptr);

  BehaviorModelPtr behavior_uct(new BehaviorUCTSingleAgentMacroActions(params));
  world->GetAgents().begin()->second->SetBehaviorModel(behavior_uct);

  auto evaluator_drivable_area = EvaluatorDrivableArea();
  auto evaluator_collision_ego = EvaluatorCollisionEgoAgent(world->GetAgents().begin()->second->GetAgentId());

  bool goal_reached = false;
  for(int i =0; i<100; ++i) {
    world->Step(prediction_time_span);
    bool outside_drivable_area = boost::get<bool>(evaluator_drivable_area.Evaluate(*world));
    bool collision_ego = boost::get<bool>(evaluator_collision_ego.Evaluate(*world));
    EXPECT_FALSE(outside_drivable_area);
    EXPECT_FALSE(collision_ego);
    LOG(INFO) << world->GetAgents().begin()->second->GetCurrentState();
    if(world->GetAgents().begin()->second->AtGoal()) {
      goal_reached = true;
      break;
    }
  }
  EXPECT_TRUE(goal_reached);

}



int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();

}