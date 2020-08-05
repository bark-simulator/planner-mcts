// Copyright (c) 2020 Julian Bernhard
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.
#include <chrono>
#include "gtest/gtest.h"
#include "bark_mcts/models/behavior/behavior_uct_hypothesis.hpp"
#include "bark/models/behavior/motion_primitives/param_config/behav_macro_actions_from_param_server.hpp"
#include "bark_mcts/models/behavior/param_config/mcts_parameters_from_param_server.hpp"
#include "bark_mcts/models/behavior/param_config/mcts_state_parameters_from_param_server.hpp"
#include "bark_mcts/models/behavior/tests/test_helpers.hpp"
#include "bark/commons/params/setter_params.hpp"

#include "bark/world/tests/make_test_world.hpp"
#include "bark/world/tests/make_test_xodr_map.hpp"
#include "bark/models/behavior/motion_primitives/motion_primitives.hpp"
#include "bark/models/behavior/motion_primitives/macro_actions.hpp"
#include "bark/models/behavior/constant_velocity/constant_velocity.hpp"
#include "bark_mcts/models/behavior/hypothesis/idm/hypothesis_idm.hpp"
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

ParamsPtr make_params_hypothesis(float headway_lower, float headway_upper, float fixed_headway,
                                 float acc_lower_bound=-5.0f, float acc_upper_bound=8.0f,
                                 float buckets_lower_bound = -8.0f, float buckets_upper_bound=9.0f) {
    // Behavior params
    auto params = std::make_shared<SetterParams>(false);
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
    params->SetReal("BehaviorIDMClassic::CoolnessFactor", 0.0f);
    // IDM Stochastic Headway
    params->SetInt("BehaviorIDMStochastic::HeadwayDistribution::RandomSeed", 1234);
    params->SetReal("BehaviorIDMStochastic::HeadwayDistribution::LowerBound", headway_lower);
    params->SetReal("BehaviorIDMStochastic::HeadwayDistribution::UpperBound", headway_upper);
    params->SetDistribution("BehaviorIDMStochastic::HeadwayDistribution", "UniformDistribution1D");

    params->SetDistribution("BehaviorIDMStochastic::SpacingDistribution", "FixedValue");
    params->SetListFloat("BehaviorIDMStochastic::SpacingDistribution::FixedValue", {0.0f});
    params->SetDistribution("BehaviorIDMStochastic::MaxAccDistribution", "FixedValue");
    params->SetListFloat("BehaviorIDMStochastic::MaxAccDistribution::FixedValue", {1.0f});
    params->SetDistribution("BehaviorIDMStochastic::DesiredVelDistribution", "FixedValue");
    params->SetListFloat("BehaviorIDMStochastic::DesiredVelDistribution::FixedValue", {15.0f});
    params->SetDistribution("BehaviorIDMStochastic::ComftBrakingDistribution", "FixedValue");
    params->SetListFloat("BehaviorIDMStochastic::ComftBrakingDistribution::FixedValue", {1.0f});
    params->SetDistribution("BehaviorIDMStochastic::CoolnessFactorDistribution", "FixedValue");
    params->SetListFloat("BehaviorIDMStochastic::CoolnessFactorDistribution::FixedValue", {0.0f});
    // IDM Hypothesis
    params->SetInt("BehaviorHypothesisIDMStochastic::NumSamples", 100000);
    params->SetInt("BehaviorHypothesisIDMStochastic::NumBuckets", 1000);
    params->SetReal("BehaviorHypothesisIDMStochastic::BucketsLowerBound", buckets_lower_bound);
    params->SetReal("BehaviorHypothesisIDMStochastic::BucketsUpperBound", buckets_upper_bound);

    return params;
}

TEST(hypothesis_mcts_state, execute) {
  // Setup prediction models for ego agent and other agents   
  auto params = std::make_shared<SetterParams>(false);

  params->SetReal("Mcts::State::GoalReward", 200.0);
  params->SetReal("Mcts::State::CollisionReward", -2000.0);
  params->SetReal("Mcts::State::GoalCost", 10.0);
  params->SetReal("Mcts::State::CollisionCost", 300);
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
  auto const_observed_world = std::const_pointer_cast<ObservedWorld>(observed_world);
  const auto num_ego_actions = std::dynamic_pointer_cast<BehaviorMotionPrimitives>(ego_behavior_model)->GetNumMotionPrimitives(const_observed_world);

  // Init hypothesis mcts state
  mcts::HypothesisBeliefTracker belief_tracker(MctsParametersFromParamServer(params));
  auto params_hyp1 = make_params_hypothesis(1.0, 1.5, 1.5);
  auto params_hyp2 = make_params_hypothesis(1.5, 3.0, 1.5);
  std::vector<BehaviorModelPtr> behavior_hypothesis;
  behavior_hypothesis.push_back(
          std::make_shared<BehaviorHypothesisIDM>(params_hyp1));
  behavior_hypothesis.push_back(
          std::make_shared<BehaviorHypothesisIDM>(params_hyp2));

  auto ego_agent_id = observed_world->GetAgents().begin()->first;
  auto front_agent_id = std::next(observed_world->GetAgents().begin())->first;
  auto state_params = MctsStateParametersFromParamServer(params);
  MctsStateHypothesis<> mcts_state(const_observed_world,
                        false,
                       num_ego_actions,
                       prediction_time_span,
                       belief_tracker.sample_current_hypothesis(),
                       behavior_hypothesis,
                       ego_behavior_model,
                       ego_agent_id,
                       state_params);
  
  std::vector<mcts::Reward> rewards;
  mcts::Cost cost;
  belief_tracker.belief_update(mcts_state, mcts_state);
  belief_tracker.sample_current_hypothesis();
  auto t1 = std::chrono::high_resolution_clock::now();
  auto action_idx = mcts_state.plan_action_current_hypothesis(front_agent_id);
  auto t2 = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::microseconds>( t2 - t1 ).count();
  std::cout << "Duration" << duration << "[ms]";
  auto next_mcts_state = mcts_state.execute(JointAction({0, action_idx}), rewards, cost);

  // Check Get Last Action other agent
  auto last_action_state = mcts_state.get_last_action(front_agent_id);
  auto last_action_plan = observed_world->GetAgent(front_agent_id)->GetBehaviorModel()->GetLastAction();
  EXPECT_TRUE(last_action_state  == last_action_plan);

  // Check Get Probability -> only interface, calculation is checked in hypothesis
  LOG(INFO) << "Warning due to bucket bound can be neglected. We check for an out-of-bound action.";
  auto probability = next_mcts_state->get_probability(0, front_agent_id, Action(20.0f));
  EXPECT_EQ(probability, 0);

  // Checking reward of zero if we are neither colliding or at goal
  EXPECT_FALSE(next_mcts_state->is_terminal()); // < make test world is defined in such a way that
                                                // a long driving corridor along x exists
                                                // no collision should occur after one action
  EXPECT_NEAR(rewards[0], 0.0f , 0.00001);
  EXPECT_NEAR(cost, 0,0.001);

  // Checking collision with other agent
  next_mcts_state = mcts_state.execute(JointAction({2, action_idx}), rewards, cost);
  auto reached = next_mcts_state->is_terminal();
  for (int i = 0; i < 1000; ++i) {
    LOG(INFO) << "---------------\n" << next_mcts_state->sprintf();
    if(next_mcts_state->is_terminal()) {
      reached = true;
      break;
    }
    auto action_idx2 = next_mcts_state->plan_action_current_hypothesis(1);
    next_mcts_state = next_mcts_state->execute(JointAction({2, action_idx2}), rewards, cost);
  }
  EXPECT_TRUE(next_mcts_state->is_terminal()); // < acceleration should lead to a collision with other agent
  EXPECT_NEAR(rewards[0], params->GetReal("Mcts::State::CollisionReward", "", 1.0) , 0.00001);
  EXPECT_NEAR(cost, params->GetReal("Mcts::State::CollisionCost", "", 1.0) , 0.00001);


  // Checking goal reached: Do multiple steps and expect that goal is reached
  next_mcts_state = mcts_state.execute(JointAction({0, action_idx}), rewards, cost);
  for (int i = 0; i < 1000; ++i) {
    if(next_mcts_state->is_terminal()) {
      break;
    }
    auto action_idx2 = next_mcts_state->plan_action_current_hypothesis(1);
    next_mcts_state = next_mcts_state->execute(JointAction({0, action_idx2}), rewards, cost);
  }
  EXPECT_NEAR(rewards[0], params->GetReal("Mcts::State::GoalReward", "", 1.0)  , 0.00001); // < reward should be one when reaching the goal 
  EXPECT_NEAR(cost, params->GetReal("Mcts::State::GoalCost", "", 1.0) , 0.00001);
}


TEST(behavior_uct_single_agent_macro_actions, no_agent_in_front_accelerate) {
  // Test if uct planner accelerates if there is no agent in front
  auto params = std::make_shared<SetterParams>();

  float ego_velocity = 2.0, rel_distance = 7.0, velocity_difference=0.0, prediction_time_span=0.5f;
  Polygon polygon(Pose(1, 1, 0), std::vector<Point2d>{Point2d(-3, 3), Point2d(-3, 3), Point2d(3, 3), Point2d(3, -3), Point2d(-3, -3)});
  std::shared_ptr<Polygon> goal_polygon(std::dynamic_pointer_cast<Polygon>(polygon.Translate(Point2d(150, -1.75)))); // < move the goal polygon into the driving corridor in front of the ego vehicle
  auto goal_definition_ptr = std::make_shared<GoalDefinitionPolygon>(*goal_polygon);
  
  auto observed_world = make_test_observed_world(0,rel_distance, ego_velocity, velocity_difference, goal_definition_ptr);
  observed_world.SetRemoveAgents(true);
  auto params_hyp1 = make_params_hypothesis(1.0, 1.5, 1.5);
  auto params_hyp2 = make_params_hypothesis(1.5, 3.0, 1.5);
  std::vector<BehaviorModelPtr> behavior_hypothesis;
  behavior_hypothesis.push_back(
          std::make_shared<BehaviorHypothesisIDM>(params_hyp1));
  behavior_hypothesis.push_back(
          std::make_shared<BehaviorHypothesisIDM>(params_hyp2));

  bark::models::behavior::BehaviorUCTHypothesis behavior_uct(params, behavior_hypothesis);

  Trajectory trajectory = behavior_uct.Plan(prediction_time_span, observed_world);
  auto action = behavior_uct.GetLastAction();
  EXPECT_TRUE(boost::get<Continuous1DAction>(action)>= 0.0); // << max, available acceleration is action 2
}

TEST(behavior_uct_single_agent, agent_in_front_must_brake) {
  // Test if uct planner brakes when slow agent is directly in front
  auto params = std::make_shared<SetterParams>();

  float ego_velocity = 5.0, rel_distance = 2.0, velocity_difference=2.0, prediction_time_span=0.5f;
  Polygon polygon(Pose(1, 1, 0), std::vector<Point2d>{Point2d(-3, 3), Point2d(-3, 3), Point2d(3, 3), Point2d(3, -3), Point2d(-3, -3)});
  std::shared_ptr<Polygon> goal_polygon(std::dynamic_pointer_cast<Polygon>(polygon.Translate(Point2d(150, -1.75)))); // < move the goal polygon into the driving corridor in front of the ego vehicle
  auto goal_definition_ptr = std::make_shared<GoalDefinitionPolygon>(*goal_polygon);
  
  auto observed_world = make_test_observed_world(2,rel_distance, ego_velocity, velocity_difference, goal_definition_ptr);
  observed_world.SetRemoveAgents(true);
  auto params_hyp1 = make_params_hypothesis(1.0, 1.5, 1.5);
  auto params_hyp2 = make_params_hypothesis(1.5, 3.0, 1.5);
  std::vector<BehaviorModelPtr> behavior_hypothesis;
  behavior_hypothesis.push_back(
          std::make_shared<BehaviorHypothesisIDM>(params_hyp1));
  behavior_hypothesis.push_back(
          std::make_shared<BehaviorHypothesisIDM>(params_hyp2));
  bark::models::behavior::BehaviorUCTHypothesis behavior_uct(params, behavior_hypothesis);

  Trajectory trajectory = behavior_uct.Plan(prediction_time_span, observed_world);
  auto action = behavior_uct.GetLastAction();
  EXPECT_TRUE(boost::get<Continuous1DAction>(action) < 0.0f); // some decceleration should occur
}

TEST(behavior_uct_single_agent, change_lane) {
  // Test if the planner reaches the goal at some point when agent is slower and in front
  auto params = std::make_shared<SetterParams>(false);

  // Desired headway should correspond to initial headway
  params->SetInt("BehaviorUctHypothesis::Mcts::MaxNumIterations", 400);
  params->SetInt("BehaviorUctHypothesis::Mcts::MaxSearchTime", 4000);
  params->SetInt("BehaviorUctHypothesis::Mcts::RandomSeed", 1000);
  params->SetBool("BehaviorUctHypothesis::DumpTree", true);
  params->SetListFloat("BehaviorUctHypothesis::EgoBehavior::AccelerationInputs", {0, 1, 4, -1, -8});
  params->SetReal("BehaviorUctHypothesis::Mcts::DiscountFactor", 0.9);
  params->SetReal("BehaviorUctHypothesis::Mcts::UctStatistic::ExplorationConstant", 0.7);
  params->SetInt("BehaviorUctHypothesis::Mcts::RandomHeuristic::MaxSearchTime", 20000);
  params->SetInt("BehaviorUctHypothesis::Mcts::RandomHeuristic::MaxNumIterations", 10);
  params->SetReal("BehaviorUctHypothesis::Mcts::UctStatistic::ReturnLowerBound", -1000);
  params->SetReal("BehaviorUctHypothesis::Mcts::UctStatistic::ReturnUpperBound", 100);


  // IDM Classic
  params->SetReal("BehaviorIDMClassic::MinimumSpacing", 0.0f); // Required for testing
  params->SetReal("BehaviorIDMClassic::DesiredTimeHeadway", 1.5);
  params->SetReal("BehaviorIDMClassic::MaxAcceleration", 1.0f); // Required for testing
  params->SetReal("BehaviorIDMClassic::AccelerationLowerBound", -8.0);
  params->SetReal("BehaviorIDMClassic::AccelerationUpperBound", 5.0);
  params->SetReal("BehaviorIDMClassic::DesiredVelocity", 15.0f);
  params->SetReal("BehaviorIDMClassic::ComfortableBrakingAcceleration",  1.0f);
  params->SetReal("BehaviorIDMClassic::MinVelocity", 0.0f);
  params->SetReal("BehaviorIDMClassic::MaxVelocity", 50.0f);
  params->SetInt("BehaviorIDMClassic::Exponent", 4);
  params->SetReal("BehaviorIDMClassic::CoolnessFactor", 0.0f);
  // IDM Stochastic
  params->SetInt("BehaviorIDMStochastic::HeadwayDistribution::RandomSeed", 1234);
  params->SetReal("BehaviorIDMStochastic::HeadwayDistribution::LowerBound", 1);
  params->SetReal("BehaviorIDMStochastic::HeadwayDistribution::UpperBound", 2);
  params->SetDistribution("BehaviorIDMStochastic::HeadwayDistribution", "UniformDistribution1D");

  params->SetDistribution("BehaviorIDMStochastic::SpacingDistribution", "FixedValue");
  params->SetListFloat("BehaviorIDMStochastic::SpacingDistribution::FixedValue", {0.0f});
  params->SetDistribution("BehaviorIDMStochastic::MaxAccDistribution", "FixedValue");
  params->SetListFloat("BehaviorIDMStochastic::MaxAccDistribution::FixedValue", {1.0f});
  params->SetDistribution("BehaviorIDMStochastic::DesiredVelDistribution", "FixedValue");
  params->SetListFloat("BehaviorIDMStochastic::DesiredVelDistribution::FixedValue", {15.0f});
  params->SetDistribution("BehaviorIDMStochastic::ComftBrakingDistribution", "FixedValue");
  params->SetListFloat("BehaviorIDMStochastic::ComftBrakingDistribution::FixedValue", {1.0f});
  params->SetDistribution("BehaviorIDMStochastic::CoolnessFactorDistribution", "FixedValue");
  params->SetListFloat("BehaviorIDMStochastic::CoolnessFactorDistribution::FixedValue", {0.0f});
  // IDM Hypothesis
  params->SetInt("BehaviorHypothesisIDMStochastic::NumSamples", 1000000);
  params->SetInt("BehaviorHypothesisIDMStochastic::NumBuckets", 1000);
  params->SetReal("BehaviorHypothesisIDMStochastic::BucketsLowerBound", -9.0);
  params->SetReal("BehaviorHypothesisIDMStochastic::BucketsUpperBound", 6.0);

  // Map creation
  OpenDriveMapPtr open_drive_map = MakeXodrMapOneRoadTwoLanes();
  MapInterfacePtr map_interface = std::make_shared<MapInterface>();
  map_interface->interface_from_opendrive(open_drive_map);

  // Hypothesis behavior creation
  auto ego_behavior_model = BehaviorMacroActionsFromParamServer(
                                              params);
  auto make_hyp_params = [&](float lower, float upper) {
    auto params_new = std::make_shared<SetterParams>(false, params->GetCondensedParamList());
    params_new->SetReal("BehaviorIDMStochastic::HeadwayDistribution::LowerBound", lower);
    params_new->SetReal("BehaviorIDMStochastic::HeadwayDistribution::UpperBound", upper);
    return params_new;
  };
                                          
  auto params_hyp1 = make_hyp_params(1, 1.5);
  auto params_hyp2 = make_hyp_params(1.5, 2.0);
  std::vector<BehaviorModelPtr> behavior_hypothesis;
  behavior_hypothesis.push_back(
          std::make_shared<BehaviorHypothesisIDM>(params_hyp1));
  behavior_hypothesis.push_back(
          std::make_shared<BehaviorHypothesisIDM>(params_hyp2));


  auto behavior_uct = std::make_shared<BehaviorUCTHypothesis>(params, behavior_hypothesis);

  // Agent and world Creation
  auto ego_agent = CreateAgent(true, 5.0, 15.0, false, params, map_interface);
  auto left_agent1 = CreateAgent(false, 3.0, 14.0, false, params, map_interface);
  auto left_agent2 = CreateAgent(false, 3.0+4.0+10.0, 14.0, false, params, map_interface);

  WorldPtr world(new World(params));
  ego_agent->SetBehaviorModel(behavior_uct);
  world->AddAgent(ego_agent);
  world->AddAgent(left_agent1);
  world->AddAgent(left_agent2);
  world->UpdateAgentRTree();

  world->SetMap(map_interface);

  // Run simulation with evaluations
  auto evaluator_drivable_area = EvaluatorDrivableArea();
  auto evaluator_collision_ego = EvaluatorCollisionEgoAgent(world->GetAgents().begin()->second->GetAgentId());

  bool goal_reached = false;
  for(int i =0; i<100; ++i) {
    world->Step(0.2);
    bool outside_drivable_area = boost::get<bool>(evaluator_drivable_area.Evaluate(*world));
    bool collision_ego = boost::get<bool>(evaluator_collision_ego.Evaluate(*world));
    EXPECT_FALSE(outside_drivable_area);
    EXPECT_FALSE(collision_ego);

    LOG(INFO) << "Time step " << i*0.2;
    for (const auto& agent : world->GetAgents()) {
      LOG(INFO) << "Agent " << agent.first << ", State: " << agent.second->GetCurrentState() << 
          ", Action: " <<  boost::apply_visitor(action_tostring_visitor(), agent.second->GetBehaviorModel()->GetLastAction()) ;
    }
    LOG(INFO) << behavior_uct->GetBeliefTracker().sprintf();

    if(world->GetAgents().begin()->second->AtGoal()) {
      goal_reached = true;
      break;
    }
  }
  EXPECT_TRUE(goal_reached);
}

TEST(behavior_uct_single_agent, belief_test) {
  // Test if the planner reaches the goal at some point when agent is slower and in front
  auto params = std::make_shared<SetterParams>(false);

  // Desired headway should correspond to initial headway
  params->SetInt("BehaviorUctHypothesis::Mcts::MaxNumIterations", 400);
  params->SetInt("BehaviorUctHypothesis::Mcts::MaxSearchTime", 4000);
  params->SetInt("BehaviorUctHypothesis::Mcts::RandomSeed", 1000);
  params->SetBool("BehaviorUctHypothesis::DumpTree", true);
  params->SetListListFloat("BehaviorUctHypothesis::MotionPrimitiveInputs", {{0,0}, {1,0}, {0,-0.27}, {0, 0.27}, {0,-0.17}, {0, 0.17}, {-1,0}}); 
  params->SetReal("BehaviorUctHypothesis::Mcts::DiscountFactor", 0.9);
  params->SetReal("BehaviorUctHypothesis::Mcts::UctStatistic::ExplorationConstant", 0.7);
  params->SetInt("BehaviorUctHypothesis::Mcts::RandomHeuristic::MaxSearchTime", 20000);
  params->SetInt("BehaviorUctHypothesis::Mcts::RandomHeuristic::MaxNumIterations", 10);
  params->SetReal("BehaviorUctHypothesis::Mcts::UctStatistic::ReturnLowerBound", -1000);
  params->SetReal("BehaviorUctHypothesis::Mcts::UctStatistic::ReturnUpperBound", 100);

  params->SetReal("BehaviorUctHypothesis::Mcts::State::GoalReward", 100.0);
  params->SetReal("BehaviorUctHypothesis::Mcts::State::CollisionReward", -1000.0);
  params->SetReal("BehaviorUctHypothesis::Mcts::State::GoalCost", 0.0);
  params->SetReal("BehaviorUctHypothesis::Mcts::State::CollisionCost", 1000);

  // IDM Classic
  params->SetReal("BehaviorIDMClassic::MinimumSpacing", 1.0f); // Required for testing
  params->SetReal("BehaviorIDMClassic::DesiredTimeHeadway", 1.5);
  params->SetReal("BehaviorIDMClassic::MaxAcceleration", 1.0f); // Required for testing
  params->SetReal("BehaviorIDMClassic::AccelerationLowerBound", -8.0);
  params->SetReal("BehaviorIDMClassic::AccelerationUpperBound", 5.0);
  params->SetReal("BehaviorIDMClassic::DesiredVelocity", 8.0f);
  params->SetReal("BehaviorIDMClassic::ComfortableBrakingAcceleration",  1.0f);
  params->SetReal("BehaviorIDMClassic::MinVelocity", 0.0f);
  params->SetReal("BehaviorIDMClassic::MaxVelocity", 50.0f);
  params->SetInt("BehaviorIDMClassic::Exponent", 4);
  params->SetReal("BehaviorIDMClassic::CoolnessFactor", 0.0f);
  // IDM Stochastic Headway
  params->SetInt("BehaviorIDMStochastic::HeadwayDistribution::RandomSeed", 1234);
  params->SetReal("BehaviorIDMStochastic::HeadwayDistribution::LowerBound", 1.0);
  params->SetReal("BehaviorIDMStochastic::HeadwayDistribution::UpperBound", 3.0);
  params->SetDistribution("BehaviorIDMStochastic::HeadwayDistribution", "UniformDistribution1D");

  params->SetDistribution("BehaviorIDMStochastic::SpacingDistribution", "FixedValue");
  params->SetListFloat("BehaviorIDMStochastic::SpacingDistribution::FixedValue", {0.0f});
  params->SetDistribution("BehaviorIDMStochastic::MaxAccDistribution", "FixedValue");
  params->SetListFloat("BehaviorIDMStochastic::MaxAccDistribution::FixedValue", {1.0f});
  params->SetDistribution("BehaviorIDMStochastic::DesiredVelDistribution", "FixedValue");
  params->SetListFloat("BehaviorIDMStochastic::DesiredVelDistribution::FixedValue", {15.0f});
  params->SetDistribution("BehaviorIDMStochastic::ComftBrakingDistribution", "FixedValue");
  params->SetListFloat("BehaviorIDMStochastic::ComftBrakingDistribution::FixedValue", {1.0f});
  params->SetDistribution("BehaviorIDMStochastic::CoolnessFactorDistribution", "FixedValue");
  params->SetListFloat("BehaviorIDMStochastic::CoolnessFactorDistribution::FixedValue", {0.0f});

  // IDM Hypothesis
  params->SetInt("BehaviorHypothesisIDMStochasticHeadway::NumSamples", 1000000);
  params->SetInt("BehaviorHypothesisIDMStochasticHeadway::NumBuckets", 1000);
  params->SetReal("BehaviorHypothesisIDMStochasticHeadway::BucketsLowerBound", -9.0);
  params->SetReal("BehaviorHypothesisIDMStochasticHeadway::BucketsUpperBound", 6.0);


  // Map creation
  OpenDriveMapPtr open_drive_map = MakeXodrMapOneRoadTwoLanes();
  MapInterfacePtr map_interface = std::make_shared<MapInterface>();
  map_interface->interface_from_opendrive(open_drive_map);

  // Hypothesis behavior creation
  auto ego_behavior_model = BehaviorMacroActionsFromParamServer(
                                              params);
  auto make_hyp_params = [&](float lower, float upper) {
    auto params_new = std::make_shared<SetterParams>(false, params->GetCondensedParamList());
    params_new->SetReal("BehaviorIDMStochastic::HeadwayDistribution::LowerBound", lower);
    params_new->SetReal("BehaviorIDMStochastic::HeadwayDistribution::UpperBound", upper);
    return params_new;
  };

  auto params_hyp1 = make_hyp_params(1, 2);
  auto params_hyp2 = make_hyp_params(2, 3);
  auto params_hyp3 = make_hyp_params(3, 4);
  std::vector<BehaviorModelPtr> behavior_hypothesis;
  behavior_hypothesis.push_back(
          std::make_shared<BehaviorHypothesisIDM>(params_hyp1));
  behavior_hypothesis.push_back(
          std::make_shared<BehaviorHypothesisIDM>(params_hyp2));
  behavior_hypothesis.push_back(
          std::make_shared<BehaviorHypothesisIDM>(params_hyp3));



  // Agent and world Creation
  auto ego_agent = CreateAgent(true, 5.0, 2.7, false, params, map_interface);
  auto left_agent1 = CreateAgent(false, 3.0, 5, false, params, map_interface);
  auto left_agent2 = CreateAgent(false, 3.0+4.0+0.5, 5, false, params, map_interface);

  WorldPtr world(new World(params));
  world->AddAgent(ego_agent);
  world->AddAgent(left_agent1);
  world->AddAgent(left_agent2);
  world->UpdateAgentRTree();

  world->SetMap(map_interface);

  mcts::HypothesisBeliefTracker belief_tracker(
                      bark::models::behavior::MctsParametersFromParamServer(
                                      params->AddChild("BehaviorUctHypothesis")));

  auto observed_world = std::make_shared<ObservedWorld>(world->Clone(), ego_agent->GetAgentId());
  auto const_observed_world = std::const_pointer_cast<ObservedWorld>(observed_world);
  auto state_params = MctsStateParametersFromParamServer(params);
  auto last_mcts_state = std::make_shared<MctsStateHypothesis<>>(const_observed_world,
                        false,
                       2,
                       0.2,
                       belief_tracker.sample_current_hypothesis(),
                       behavior_hypothesis,
                       nullptr,
                       ego_agent->GetAgentId(),
                       state_params);

  belief_tracker.belief_update(*last_mcts_state, *last_mcts_state);

  for(int i =0; i<10; ++i) {
    world->Step(0.2);

    auto next_world = std::make_shared<ObservedWorld>(world->Clone(), ego_agent->GetAgentId());
    auto next_const_observed_world = std::const_pointer_cast<ObservedWorld>(next_world);
    auto state_params = MctsStateParametersFromParamServer(params);
    auto mcts_state = std::make_shared<MctsStateHypothesis<>>(next_const_observed_world,
                      false,
                       2,
                       0.2,
                       belief_tracker.sample_current_hypothesis(),
                       behavior_hypothesis,
                       nullptr,
                       ego_agent->GetAgentId(),
                       state_params);
    belief_tracker.belief_update(*last_mcts_state, *mcts_state);
    last_mcts_state = mcts_state;

    LOG(INFO) << "Time step " << i*0.2;
    for (const auto& agent : world->GetAgents()) {
      LOG(INFO) << "Agent " << agent.first << ", State: " << agent.second->GetCurrentState() << 
          ", Action: " << boost::apply_visitor(action_tostring_visitor(), agent.second->GetBehaviorModel()->GetLastAction());
    }
    LOG(INFO) << belief_tracker.sprintf();
  }

  EXPECT_NEAR(belief_tracker.get_beliefs()
              .at(left_agent1->GetAgentId())[0], 0.5, 0.1);
  EXPECT_NEAR(belief_tracker.get_beliefs()
              .at(left_agent1->GetAgentId())[1], 0.5, 0.1);
  EXPECT_NEAR(belief_tracker.get_beliefs()
              .at(left_agent1->GetAgentId())[2], 0.0, 0.1);

  // This agent was not influenced by parameter sampling all hypothesis should be equal likley
  EXPECT_NEAR(belief_tracker.get_beliefs()
              .at(left_agent2->GetAgentId())[0], 0.333, 0.08);
  EXPECT_NEAR(belief_tracker.get_beliefs()
              .at(left_agent2->GetAgentId())[1], 0.333, 0.08);
  EXPECT_NEAR(belief_tracker.get_beliefs()
              .at(left_agent2->GetAgentId())[2], 0.333, 0.08);
}

TEST(behavior_uct_single_agent_macro_actions, normalization) {
  // Try to provoke a normalization error
  auto params = std::make_shared<SetterParams>(false);
  params->SetInt("BehaviorUctHypothesis::Mcts::MaxNumIterations", 20000);
  params->SetInt("BehaviorUctHypothesis::Mcts::MaxSearchTime", 20000);
  params->SetInt("BehaviorUctHypothesis::Mcts::RandomSeed", 1000);
  params->SetBool("BehaviorUctHypothesis::DumpTree", true);
  params->SetListListFloat("BehaviorUctHypothesis::MotionPrimitiveInputs", {{0,0}, {1,0}, {0,-0.27}, {0, 0.27}, {0,-0.17}, {0, 0.17}, {-1,0}}); 
  params->SetReal("BehaviorUctHypothesis::Mcts::DiscountFactor", 0.9);
  params->SetReal("BehaviorUctHypothesis::Mcts::UctStatistic::ExplorationConstant", 0.7);
  params->SetInt("BehaviorUctHypothesis::Mcts::RandomHeuristic::MaxSearchTime", 20000);
  params->SetInt("BehaviorUctHypothesis::Mcts::RandomHeuristic::MaxNumIterations", 40);
  params->SetReal("BehaviorUctHypothesis::Mcts::UctStatistic::ReturnLowerBound", -1000);
  params->SetReal("BehaviorUctHypothesis::Mcts::UctStatistic::ReturnUpperBound", 100);

  params->SetBool("BehaviorUctHypothesis::Mcts::HypothesisStatistic::CostBasedActionSelection", true);
  params->SetReal("BehaviorUctHypothesis::Mcts::HypothesisStatistic::LowerCostBound", -100);
  params->SetReal("BehaviorUctHypothesis::Mcts::HypothesisStatistic::UpperCostBound", 1000);

  params->SetReal("BehaviorUctHypothesis::Mcts::State::GoalReward", 100.0);
  params->SetReal("BehaviorUctHypothesis::Mcts::State::CollisionReward", -1000.0);
  params->SetReal("BehaviorUctHypothesis::Mcts::State::GoalCost", -100.0);
  params->SetReal("BehaviorUctHypothesis::Mcts::State::CollisionCost", 1000);

  float ego_velocity = 2.0, rel_distance = 7.0, velocity_difference=0.0, prediction_time_span=0.5f;
  Polygon polygon(Pose(1, 1, 0), std::vector<Point2d>{Point2d(-3, 3), Point2d(-3, 3), Point2d(3, 3), Point2d(3, -3), Point2d(-3, -3)});
  std::shared_ptr<Polygon> goal_polygon(std::dynamic_pointer_cast<Polygon>(polygon.Translate(Point2d(5, -1.75)))); // < move the goal polygon into the driving corridor in front of the ego vehicle
  auto goal_definition_ptr = std::make_shared<GoalDefinitionPolygon>(*goal_polygon);

  auto observed_world = make_test_observed_world(2,rel_distance, ego_velocity, velocity_difference, goal_definition_ptr);
  observed_world.SetRemoveAgents(true);
  auto params_hyp1 = make_params_hypothesis(1.0, 1.5, 1.5);
  auto params_hyp2 = make_params_hypothesis(1.5, 3.0, 1.5);
  std::vector<BehaviorModelPtr> behavior_hypothesis;
  behavior_hypothesis.push_back(
          std::make_shared<BehaviorHypothesisIDM>(params_hyp1));
  behavior_hypothesis.push_back(
          std::make_shared<BehaviorHypothesisIDM>(params_hyp2));

  bark::models::behavior::BehaviorUCTHypothesis behavior_uct(params, behavior_hypothesis);

  try {
     Trajectory trajectory = behavior_uct.Plan(prediction_time_span, observed_world);
  } catch(...) {
    EXPECT_TRUE(false);
  }
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();

}