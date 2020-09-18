// Copyright (c) 2020 Julian Bernhard
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.
#include <chrono>
#include "gtest/gtest.h"
#include "mcts/hypothesis/hypothesis_belief_tracker.h"
#include "bark_mcts/models/behavior/behavior_uct_risk_constraint.hpp"
#include "bark_mcts/models/behavior/mcts_state/mcts_state_risk_constraint.hpp"
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
    params->SetInt("BehaviorHypothesisIDM::NumSamples", 200);
    params->SetInt("BehaviorHypothesisIDM::NumBuckets", 10);
    params->SetReal("BehaviorHypothesisIDMStochastic::BucketsLowerBound", buckets_lower_bound);
    params->SetReal("BehaviorHypothesisIDMStochastic::BucketsUpperBound", buckets_upper_bound);

    return params;
}

TEST(risk_constraint_mcts_state, execute) {
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
  auto sampling_prob1 = std::dynamic_pointer_cast<BehaviorHypothesis>(behavior_hypothesis[0])
                           ->ParameterSamplingProbability();
  EXPECT_NEAR(sampling_prob1, 1/(1.5 - 1.0), 0.0001);
  
  auto front_agent_id = std::next(observed_world->GetAgents().begin())->first;
  auto state_params = MctsStateParametersFromParamServer(params);
  const auto& current_beliefs = belief_tracker.get_beliefs();
  MctsStateRiskConstraint mcts_state(const_observed_world,
                        false,
                       num_ego_actions,
                       prediction_time_span,
                       belief_tracker.sample_current_hypothesis(),
                       behavior_hypothesis,
                       ego_behavior_model,
                       ego_agent_id,
                       state_params,
                       current_beliefs,
                       1.0f);

  std::vector<mcts::Reward> rewards;
  mcts::Cost cost;
  belief_tracker.belief_update(mcts_state, mcts_state);
  belief_tracker.sample_current_hypothesis();
  auto t1 = std::chrono::high_resolution_clock::now();
  auto action_idx = mcts_state.plan_action_current_hypothesis(front_agent_id);
  auto t2 = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::milliseconds>( t2 - t1 ).count();
  std::cout << "Duration" << duration << "[ms]";
  auto next_mcts_state = mcts_state.execute(JointAction({0, action_idx}), rewards, cost);

  // Checking reward of zero if we are neither colliding or at goal
  EXPECT_FALSE(next_mcts_state->is_terminal()); // < make test world is defined in such a way that
                                                // a long driving corridor along x exists
                                                // no collision should occur after one action
  EXPECT_NEAR(rewards[0], 0.0f , 0.00001);

  // Checking collision with other agent and probability calculation
  next_mcts_state = mcts_state.execute(JointAction({2, action_idx}), rewards, cost);
  decltype(next_mcts_state) last_mcts_state;
  bark::commons::Probability current_state_prob = next_mcts_state->get_state_sequence_probability();
  auto reached = next_mcts_state->is_terminal();
  for (int i = 0; i < 1000; ++i) {
    LOG(INFO) << "---------------\n" << next_mcts_state->sprintf();
    if(next_mcts_state->is_terminal()) {
      reached = true;
      break;
    }
    last_mcts_state = next_mcts_state;
    auto action_idx2 = next_mcts_state->plan_action_current_hypothesis(front_agent_id);
    next_mcts_state = next_mcts_state->execute(JointAction({2, action_idx2}), rewards, cost);
    auto last_bark_action = next_mcts_state->get_observed_world().GetAgents()[front_agent_id]->GetBehaviorModel()->GetLastAction();
    LOG(INFO) <<  "cost calculated: " << cost;
    bark::commons::Probability agent_action_probability = 0.0;
    for (std::size_t hyp_id = 0; hyp_id <  behavior_hypothesis.size(); ++hyp_id) {
      auto last_action_prob = std::dynamic_pointer_cast<BehaviorHypothesis>(behavior_hypothesis[hyp_id])
                           ->GetProbability(last_bark_action, last_mcts_state->get_observed_world(), front_agent_id);
      agent_action_probability += last_action_prob * belief_tracker.get_beliefs().at(front_agent_id).at(hyp_id);
    }
    //current_state_prob = current_state_prob * agent_action_probability;
    //EXPECT_NEAR(current_state_prob, next_mcts_state->get_state_sequence_probability(), 0.02);

  }
  EXPECT_TRUE(next_mcts_state->is_terminal()); // < acceleration should lead to a collision with other agent
  EXPECT_NEAR(rewards[0], params->GetReal("Mcts::State::CollisionReward", "", 1.0) , 0.00001);


  // Checking goal reached: Do multiple steps and expect that goal is reached
  next_mcts_state = mcts_state.execute(JointAction({0, action_idx}), rewards, cost);
  for (int i = 0; i < 1000; ++i) {
    if(next_mcts_state->is_terminal()) {
      break;
    }
    auto action_idx2 = next_mcts_state->plan_action_current_hypothesis(front_agent_id);
    next_mcts_state = next_mcts_state->execute(JointAction({0, action_idx2}), rewards, cost);
  }
  EXPECT_NEAR(rewards[0], params->GetReal("Mcts::State::GoalReward", "", 1.0)  , 0.00001); // < reward should be one when reaching the goal 
}

TEST(behavior_uct_single_agent_macro_actions, no_agent_in_front_accelerate) {
  // Test if uct planner accelerates if there is no agent in front
  auto params = std::make_shared<SetterParams>();
  params->SetReal("BehaviorUctBase::Mcts::State::GoalReward", 2.0);
  params->SetReal("BehaviorUctBase::Mcts::State::CollisionReward", 0.0);
  params->SetReal("BehaviorUctBase::Mcts::ReturnLowerBound", 0.0);
  params->SetReal("BehaviorUctBase::Mcts::ReturnUpperBound", 2.0);
  params->SetReal("BehaviorUctBase::Mcts::LowerCostBound", 0.0);
  params->SetReal("BehaviorUctBase::Mcts::UpperCostBound", 2.0);
  params->SetReal("BehaviorUctRiskConstraint::DefaultAvailableRisk", 0.1f);
  params->SetBool("BehaviorUctRiskConstraint::EstimateScenarioRisk", false);
  params->SetInt("BehaviorUctBase::Mcts::RandomHeuristic::MaxSearchTime", 20000);
  params->SetInt("BehaviorUctBase::Mcts::RandomHeuristic::MaxNumIterations", 10);

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

  bark::models::behavior::BehaviorUCTRiskConstraint behavior_uct(params, behavior_hypothesis, nullptr);

  Trajectory trajectory = behavior_uct.Plan(prediction_time_span, observed_world);
  auto action = behavior_uct.GetLastAction();
  EXPECT_TRUE(boost::get<Continuous1DAction>(action)>= 0.0); // << max, available acceleration is action 2
}

TEST(behavior_uct_single_agent, agent_in_front_must_brake) {
  // Test if uct planner brakes when slow agent is directly in front
  auto params = std::make_shared<SetterParams>();
  params->SetReal("BehaviorUctBase::Mcts::State::GoalReward", 2.0);
  params->SetReal("BehaviorUctBase::Mcts::State::CollisionReward", 0.0);
  params->SetReal("BehaviorUctBase::Mcts::State::CollisionCost", 100.0);
  params->SetReal("BehaviorUctBase::Mcts::ReturnLowerBound", 0.0);
  params->SetReal("BehaviorUctBase::Mcts::ReturnUpperBound", 2.0);
  params->SetReal("BehaviorUctBase::Mcts::LowerCostBound", 0.0);
  params->SetReal("BehaviorUctBase::Mcts::UpperCostBound", 100.0);
  params->SetReal("BehaviorUctRiskConstraint::DefaultAvailableRisk", 0.1f);
  params->SetBool("BehaviorUctRiskConstraint::EstimateScenarioRisk", false);
  params->SetInt("BehaviorUctBase::Mcts::RandomHeuristic::MaxSearchTime", 20000);
  params->SetInt("BehaviorUctBase::Mcts::RandomHeuristic::MaxNumIterations", 10);
  params->SetInt("BehaviorUctBase::Mcts::MaxNumIterations", 1000);
  params->SetInt("BehaviorUctBase::Mcts::MaxSearchTime", 400000);
  params->SetReal("BehaviorUctBase::Mcts::CostConstrainedStatistic::LambdaInit", 2.0f);
  params->SetReal("BehaviorUctBase::Mcts::CostConstrainedStatistic::Kappa", 10.0f);
  params->SetReal("BehaviorUctBase::Mcts::CostConstrainedStatistic::GradientUpdateScaling",  1.0f);
  params->SetReal("BehaviorUctBase::Mcts::CostConstrainedStatistic::TauGradientClip", 1.0f);
  params->SetReal("BehaviorUctBase::Mcts::CostConstrainedStatistic::ActionFilterFactor", 0.0f);

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
  bark::models::behavior::BehaviorUCTRiskConstraint behavior_uct(params, behavior_hypothesis, nullptr);

  Trajectory trajectory = behavior_uct.Plan(prediction_time_span, observed_world);
  auto action = behavior_uct.GetLastAction();
  EXPECT_TRUE(boost::get<Continuous1DAction>(action) < 0.0f); // some decceleration should occur
}



int main(int argc, char **argv) {
  FLAGS_alsologtostderr = true;
  FLAGS_v = 3;
  google::InitGoogleLogging("test");
  ::testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}