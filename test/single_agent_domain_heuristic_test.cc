// Copyright (c) 2019 fortiss GmbH
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "gtest/gtest.h"
#include "mcts/mcts.h"
#include "src/mcts_state_single_agent.hpp"
#include "src/behavior_uct_single_agent_macro_actions.hpp"
#include "modules/commons/params/default_params.hpp"
#include "modules/world/tests/make_test_world.hpp"
#include "modules/models/behavior/motion_primitives/motion_primitives.hpp"
#include "modules/models/behavior/motion_primitives/macro_actions.hpp"
#include "modules/models/behavior/constant_velocity/constant_velocity.hpp"
#include "modules/models/dynamic/single_track.hpp"
#include "modules/commons/params/default_params.hpp"
#include "modules/commons/params/setter_params.hpp"
#include "modules/world/evaluation/evaluator_drivable_area.hpp"
#include "modules/world/evaluation/evaluator_collision_ego_agent.hpp"
#include "modules/world/evaluation/evaluator_goal_reached.hpp"
#include "modules/world/goal_definition/goal_definition_polygon.hpp"
#include "modules/world/goal_definition/goal_definition_state_limits.hpp"

#include "src/domain_heuristic.hpp"


using namespace modules::models::behavior;
using namespace mcts;
using modules::world::tests::make_test_observed_world;
using modules::world::tests::make_test_world;
using modules::world::prediction::PredictionSettings;
using modules::models::dynamic::SingleTrackModel;
using modules::models::execution::ExecutionModelPtr;
using modules::models::dynamic::Input;
using modules::world::ObservedWorldPtr;
using modules::commons::DefaultParams;
using modules::commons::SetterParams;
using modules::geometry::Polygon;
using modules::geometry::Point2d;
using modules::geometry::Pose;
using modules::geometry::Distance;
using modules::world::goal_definition::GoalDefinitionPtr;
using modules::world::goal_definition::GoalDefinitionPolygon;
using modules::world::goal_definition::GoalDefinitionStateLimits;
using modules::models::dynamic::Trajectory;
using modules::world::evaluation::EvaluatorDrivableArea;
using modules::world::evaluation::EvaluatorGoalReached;
using modules::world::evaluation::EvaluatorCollisionEgoAgent;



TEST(behavior_uct_single_agent, change_lane_random_heuristic) {
  // Test if the planner reaches the goal at some point when agent is slower and in front
  auto params = std::make_shared<SetterParams>(false);
  params->SetInt("BehaviorUctSingleAgent::Mcts::MaxNumIterations", 30000);//10000
  params->SetInt("BehaviorUctSingleAgent::Mcts::MaxSearchTime", 200);//100
  params->SetInt("BehaviorUctSingleAgent::Mcts::RandomSeed", 1000);
  params->SetBool("BehaviorUctSingleAgent::DumpTree", true);
  params->SetListListFloat("BehaviorUctSingleAgent::MotionPrimitiveInputs", {{0,0}, {1,0}, {0,-0.27}, {0, 0.27}, {0,-0.17}, {0, 0.17}, {-1,0}}); 
  params->SetReal("BehaviorUctSingleAgent::Mcts::DiscountFactor", 0.95);//0.9/0.95
  params->SetReal("BehaviorUctSingleAgent::Mcts::UctStatistic::ExplorationConstant", 0.7);//0.7/0.6
  params->SetInt("BehaviorUctSingleAgent::Mcts::RandomHeuristic::MaxSearchTime", 20000);
  params->SetInt("BehaviorUctSingleAgent::Mcts::RandomHeuristic::MaxNumIterations", 100);//10

  params->SetReal("BehaviorUctSingleAgent::Mcts::UctStatistic::ReturnLowerBound", -50000);//-1000
  params->SetReal("BehaviorUctSingleAgent::Mcts::UctStatistic::ReturnUpperBound", 100);//100
  params->SetBool("BehaviorUctSingleAgent::UseRandomHeuristic", false);
  params->SetBool("BehaviorUctSingleAgent::NNHeuristic", false);


  float ego_velocity = 10.0, rel_distance = 2.0, velocity_difference=2.0, prediction_time_span=0.2f;//ego_velocity = 5.0
  Polygon polygon(Pose(0, 0, 0), std::vector<Point2d>{Point2d(0, -0.5), Point2d(0, 0.5), Point2d(20, 0.5), Point2d(20, -0.5), Point2d(0, -0.5)});//(20,1)
  std::shared_ptr<Polygon> goal_polygon(std::dynamic_pointer_cast<Polygon>(polygon.Translate(Point2d(20, -3.5)))); //(30, -0.5) < move the goal polygon into the driving corridor to the side of the ego vehicle
  auto goal_definition_ptr = std::make_shared<GoalDefinitionStateLimits>(*goal_polygon, std::make_pair<float, float>(-0.2f, 0.2f));
  
  auto world = make_test_world(0,rel_distance, ego_velocity, velocity_difference, goal_definition_ptr);

  BehaviorModelPtr behavior_uct(new BehaviorUCTSingleAgentMacroActions(params));
  world->GetAgents().begin()->second->SetBehaviorModel(behavior_uct);
  world->GetAgents().begin()->second->SetDynamicModel(DynamicModelPtr( new SingleTrackModel(params)));

  auto evaluator_drivable_area = EvaluatorDrivableArea();
  auto evaluator_collision_ego = EvaluatorCollisionEgoAgent(world->GetAgents().begin()->second->GetAgentId());

  bool goal_reached = false;
  for(int i =0; i<1000; ++i) {//i<100
    world->Step(prediction_time_span);
    bool outside_drivable_area = boost::get<bool>(evaluator_drivable_area.Evaluate(*world));
    bool collision_ego = boost::get<bool>(evaluator_collision_ego.Evaluate(*world));
    EXPECT_FALSE(outside_drivable_area);
    EXPECT_FALSE(collision_ego);
    LOG(INFO) << world->GetAgents().begin()->second->GetCurrentState();//State()
    auto current_distance = Distance(*goal_polygon, world->GetAgents().begin()->second->GetCurrentPosition());
    LOG(INFO) << "current distance = " << current_distance;
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
