// Copyright (c) 2019 fortiss GmbH
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.
#include <chrono>
#include "gtest/gtest.h"
#include "src/mcts_state_single_agent.hpp"
#include "src/behavior_uct_hypothesis.hpp"
#include "src/behav_macro_actions_from_param_server.hpp"
#include "src/mcts_parameters_from_param_server.hpp"
#include "modules/commons/params/setter_params.hpp"

#include "modules/commons/params/default_params.hpp"
#include "modules/world/tests/make_test_world.hpp"
#include "modules/models/behavior/motion_primitives/motion_primitives.hpp"
#include "modules/models/behavior/motion_primitives/macro_actions.hpp"
#include "modules/models/behavior/constant_velocity/constant_velocity.hpp"
#include "modules/models/behavior/hypothesis/idm/hypothesis_idm_stochastic_headway.hpp"
#include "modules/models/dynamic/single_track.hpp"
#include "modules/commons/params/default_params.hpp"

#include "modules/world/evaluation/evaluator_drivable_area.hpp"
#include "modules/world/evaluation/evaluator_collision_ego_agent.hpp"
#include "modules/world/evaluation/evaluator_goal_reached.hpp"
#include "modules/world/goal_definition/goal_definition_polygon.hpp"
#include "modules/world/goal_definition/goal_definition_state_limits.hpp"


using namespace modules::models::behavior;
using namespace mcts;
using modules::world::tests::make_test_observed_world;
using modules::world::tests::make_test_world;
using modules::world::prediction::PredictionSettings;
using modules::models::dynamic::SingleTrackModel;
using modules::models::dynamic::Input;
using modules::world::ObservedWorldPtr;
using modules::commons::DefaultParams;
using modules::commons::SetterParams;
using modules::commons::ParamsPtr;

using modules::geometry::Polygon;
using modules::geometry::Point2d;
using modules::geometry::Pose;
using modules::world::goal_definition::GoalDefinitionPtr;
using modules::world::goal_definition::GoalDefinitionPolygon;
using modules::world::goal_definition::GoalDefinitionStateLimits;
using modules::models::dynamic::Trajectory;
using modules::world::evaluation::EvaluatorDrivableArea;
using modules::world::evaluation::EvaluatorGoalReached;
using modules::world::evaluation::EvaluatorCollisionEgoAgent;

ParamsPtr make_params_hypothesis(float headway_lower, float headway_upper, float fixed_headway,
                                 float acc_lower_bound=-5.0f, float acc_upper_bound=8.0f,
                                 float buckets_lower_bound = -8.0f, float buckets_upper_bound=9.0f) {
    // Behavior params
    auto params = std::make_shared<SetterParams>(true);
    // IDM Classic
    params->SetReal("BehaviorIDMClassic::MinimumSpacing", 0.0f); // Required for testing
    params->SetReal("BehaviorIDMClassic::DesiredTimeHeadway", fixed_headway);
    params->SetReal("BehaviorIDMClassic::MaxAcceleration", 1.0f); // Required for testing
    params->SetReal("BehaviorIDMClassic::AccelerationLowerBound", acc_lower_bound);
    params->SetReal("BehaviorIDMClassic::AccelerationUpperBound", acc_upper_bound);
    params->SetReal("BehaviorIDMClassic::DesiredVelocity", 15.0f);
    params->SetReal("BehaviorIDMClassic::ComfortableBrakingAcceleration",  1.0f);
    params->SetReal("BehaviorIDMClassic::MinVelocity", 0.0f);
    params->SetReal("BehaviorIDMClassic::MaxVelocity", 50.0f);
    params->SetInt("BehaviorIDMClassic::Exponent", 4);
    // IDM Stochastic Headway
    params->SetInt("BehaviorIDMStochasticHeadway::HeadwayDistribution::RandomSeed", 1234);
    params->SetReal("BehaviorIDMStochasticHeadway::HeadwayDistribution::LowerBound", headway_lower);
    params->SetReal("BehaviorIDMStochasticHeadway::HeadwayDistribution::UpperBound", headway_upper);
    params->SetDistribution("BehaviorIDMStochasticHeadway::HeadwayDistribution", "UniformDistribution1D");
    // IDM Hypothesis
    params->SetInt("BehaviorHypothesisIDMStochasticHeadway::NumSamples", 100000);
    params->SetInt("BehaviorHypothesisIDMStochasticHeadway::NumBuckets", 1000);
    params->SetReal("BehaviorHypothesisIDMStochasticHeadway::BucketsLowerBound", buckets_lower_bound);
    params->SetReal("BehaviorHypothesisIDMStochasticHeadway::BucketsUpperBound", buckets_upper_bound);

    return params;
}



TEST(hypothesis_mcts_state, execute) {
  // Setup prediction models for ego agent and other agents   
  auto params = std::make_shared<SetterParams>();
  auto ego_behavior_model = BehaviorMacroActionsFromParamServer(
                                              params);

  // Create an observed world with specific goal definition and the corresponding mcts state
  Polygon polygon(Pose(1, 1, 0), std::vector<Point2d>{Point2d(-3, 3), Point2d(-3, 3), Point2d(3, 3), Point2d(3, -3), Point2d(-3, -3)});
  std::shared_ptr<Polygon> goal_polygon(std::dynamic_pointer_cast<Polygon>(polygon.Translate(Point2d(150, -1.75)))); // < move the goal polygon into the driving corridor in front of the ego vehicle
  auto goal_definition_ptr = std::make_shared<GoalDefinitionPolygon>(*goal_polygon);

  double rel_distance = 4.0f, ego_velocity = 5.0f, velocity_difference = 0.0f, prediction_time_span = 1.0f;
  auto observed_world = std::dynamic_pointer_cast<ObservedWorld>(
        make_test_observed_world(1, rel_distance, ego_velocity, velocity_difference, goal_definition_ptr).Clone());
  auto const_observed_world = std::const_pointer_cast<ObservedWorld>(observed_world);
  const auto num_ego_actions = std::dynamic_pointer_cast<BehaviorMotionPrimitives>(ego_behavior_model)->GetNumMotionPrimitives(const_observed_world);

  // Init hypothesis mcts state
  mcts::HypothesisBeliefTracker belief_tracker(MctsParametersFromParamServer(params));
  auto params_hyp1 = make_params_hypothesis(1.0, 1.5, 1.5);
  auto params_hyp2 = make_params_hypothesis(1.5, 3.0, 1.5);
  std::vector<BehaviorHypothesisPtr> behavior_hypothesis;
  behavior_hypothesis.push_back(
          std::dynamic_pointer_cast<BehaviorHypothesis>(
          std::make_shared<BehaviorHypothesisIDMStochasticHeadway>(params_hyp1)));
  behavior_hypothesis.push_back(
          std::dynamic_pointer_cast<BehaviorHypothesis>(
          std::make_shared<BehaviorHypothesisIDMStochasticHeadway>(params_hyp2)));
  auto ego_agent_id = observed_world->GetAgents().begin()->first;
  auto front_agent_id = std::next(observed_world->GetAgents().begin())->first;
  std::vector<mcts::AgentIdx> agent_ids = {ego_agent_id, front_agent_id};
  MctsStateHypothesis mcts_state(const_observed_world,
                        false,
                       num_ego_actions,
                       prediction_time_span,
                       belief_tracker.sample_current_hypothesis(),
                       behavior_hypothesis,
                       ego_behavior_model,
                       agent_ids);
  
  std::vector<mcts::Reward> rewards;
  mcts::Cost cost;
  belief_tracker.belief_update(mcts_state, mcts_state);
  belief_tracker.sample_current_hypothesis();
  auto t1 = std::chrono::high_resolution_clock::now();
  auto action_idx = mcts_state.plan_action_current_hypothesis(1);
  auto t2 = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::microseconds>( t2 - t1 ).count();
  std::cout << "Duration" << duration << "[ms]";
  auto next_mcts_state = mcts_state.execute(JointAction({0, action_idx}), rewards, cost);

  // Checking reward of zero if we are neither colliding or at goal
  EXPECT_FALSE(next_mcts_state->is_terminal()); // < make test world is defined in such a way that
                                                // a long driving corridor along x exists
                                                // no collision should occur after one action
  EXPECT_NEAR(rewards[0], 0 , 0.00001);
  EXPECT_NEAR(cost, 0 , 0.00001);

  // Checking collision with other agent
  next_mcts_state = mcts_state.execute(JointAction({2, action_idx}), rewards, cost);
  auto reached = next_mcts_state->is_terminal();
  for (int i = 0; i < 100; ++i) {
    if(next_mcts_state->is_terminal()) {
      reached = true;
      break;
    }
    auto action_idx2 = next_mcts_state->plan_action_current_hypothesis(1);
    next_mcts_state = next_mcts_state->execute(JointAction({2, action_idx2}), rewards, cost);
  }
  EXPECT_TRUE(next_mcts_state->is_terminal()); // < acceleration should lead to a collision with other agent
  EXPECT_NEAR(rewards[0], -1000 , 0.00001);
  EXPECT_NEAR(cost, -1000 , 0.00001);


  // Checking goal reached: Do multiple steps and expect that goal is reached
  next_mcts_state = mcts_state.execute(JointAction({0, action_idx}), rewards, cost);
  reached = next_mcts_state->is_terminal();
  for (int i = 0; i < 100; ++i) {
    if(next_mcts_state->is_terminal()) {
      reached = true;
      break;
    }
    auto action_idx2 = next_mcts_state->plan_action_current_hypothesis(1);
    next_mcts_state = next_mcts_state->execute(JointAction({0, action_idx2}), rewards, cost);
  }
  EXPECT_TRUE(reached);
  EXPECT_NEAR(rewards[0], 100 , 0.00001); // < reward should be one when reaching the goal */
  EXPECT_NEAR(cost, 0 , 0.00001);
}


// TEST(single_agent_mcts_state, execute_goal_reached_state_limits) {
//   // Setup prediction models for ego agent and other agents   
//   auto params = std::make_shared<DefaultParams>();
//   DynamicModelPtr dyn_model(new SingleTrackModel(params));
//   BehaviorModelPtr ego_prediction_model(new BehaviorMPContinuousActions(dyn_model, params));
//   float ego_velocity = 5.0, rel_distance = 7.0, velocity_difference=0.0, prediction_time_span=0.2f;
//   Input u1(2);  u1 << 0, 0;
//   Input u2(2);  u2 << 50, 3; //  < crazy action to drive out of the corridors
//   Input u3(2);  u3 << (rel_distance+4)*2/(prediction_time_span*prediction_time_span), 0; //  < action to drive into other agent with a single step
//                                                                                         //   (4m vehicle length assumed)
//   std::dynamic_pointer_cast<BehaviorMPContinuousActions>(ego_prediction_model)->AddMotionPrimitive(u1);
//   std::dynamic_pointer_cast<BehaviorMPContinuousActions>(ego_prediction_model)->AddMotionPrimitive(u2);
//   std::dynamic_pointer_cast<BehaviorMPContinuousActions>(ego_prediction_model)->AddMotionPrimitive(u3);
//   BehaviorModelPtr others_prediction_model(new BehaviorConstantVelocity(params));
//   PredictionSettings prediction_settings(ego_prediction_model, others_prediction_model);

//   // Create an observed world with specific goal definition and the corresponding mcts state
//   Polygon polygon(Pose(1, 1, 0), std::vector<Point2d>{Point2d(-5, -5), Point2d(-5, 5), Point2d(5, 5), Point2d(5, -5), Point2d(-5, -5)});
//   std::shared_ptr<Polygon> goal_polygon(std::dynamic_pointer_cast<Polygon>(polygon.Translate(Point2d(10,-1)))); // < move the state limit region to the front of the ego vehicle
//   auto goal_definition_ptr = std::make_shared<GoalDefinitionStateLimits>(*goal_polygon, std::make_pair<float, float>(-0.2f,0.2f));

//   auto observed_world = std::dynamic_pointer_cast<ObservedWorld>(
//     make_test_observed_world(0,rel_distance, ego_velocity, velocity_difference, goal_definition_ptr).Clone());
//   observed_world->SetupPrediction(prediction_settings);
//   auto const_observed_world = std::const_pointer_cast<ObservedWorld>(observed_world);
//   const auto num_ego_actions = std::dynamic_pointer_cast<BehaviorMotionPrimitives>(ego_prediction_model)->GetNumMotionPrimitives(const_observed_world);
//   MctsStateSingleAgent mcts_state(observed_world, false, num_ego_actions, prediction_time_span);
  
//   // Checking goal reached: Do multiple steps and expect that goal is reached
//   std::vector<mcts::Reward> rewards;
//   mcts::Cost cost;
//   bool reached = false;
//   auto next_mcts_state = mcts_state.execute(JointAction({0}), rewards, cost);
//   reached = next_mcts_state->is_terminal();
//   for (int i = 0; i < 10000; ++i) {
//     if(next_mcts_state->is_terminal()) {
//       reached = true;
//       break;
//     }
//     next_mcts_state = next_mcts_state->execute(JointAction({0}), rewards, cost);
//   }
//   EXPECT_TRUE(reached);
//   EXPECT_NEAR(rewards[0], 100 , 0.00001); // < reward should be one when reaching the goal
// }

TEST(behavior_uct_single_agent_macro_actions, no_agent_in_front_accelerate) {
  // Test if uct planner accelerates if there is no agent in front
  auto params = std::make_shared<modules::commons::SetterParams>(true);
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

 /* float ego_velocity = 2.0, rel_distance = 7.0, velocity_difference=0.0, prediction_time_span=0.5f;
  Polygon polygon(Pose(1, 1, 0), std::vector<Point2d>{Point2d(-5, -5), Point2d(-5, 5), Point2d(5, 5), Point2d(5, -5), Point2d(-5, -5)});
  std::shared_ptr<Polygon> goal_polygon(std::dynamic_pointer_cast<Polygon>(polygon.Translate(Point2d(20,0)))); // < move the goal polygon into the driving corridor in front of the ego vehicle
  auto goal_definition_ptr = std::make_shared<GoalDefinitionPolygon>(*goal_polygon);
  
  auto observed_world = make_test_observed_world(0,rel_distance, ego_velocity, velocity_difference, goal_definition_ptr);
*/
  //modules::models::behavior::BehaviorUCTHypothesis behavior_uct(params);

  //Trajectory trajectory = behavior_uct.Plan(prediction_time_span, observed_world);
}
/*
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

}*/



int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();

}