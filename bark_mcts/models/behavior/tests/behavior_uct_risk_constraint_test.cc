// Copyright (c) 2020 Julian Bernhard
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.
#include <chrono>
#include "gtest/gtest.h"
#include "mcts/hypothesis/hypothesis_belief_tracker.h"
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
    params->SetInt("BehaviorHypothesisIDMStochastic::NumSamples", 10000);
    params->SetInt("BehaviorHypothesisIDMStochastic::NumBuckets", 1000);
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
  auto duration = std::chrono::duration_cast<std::chrono::microseconds>( t2 - t1 ).count();
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
  bark::commons::Probability current_state_prob = 1.0;
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
    bark::commons::Probability desired_transition_prob = 1.0;
    for (std::size_t hyp_id = 0; hyp_id <  behavior_hypothesis.size(); ++hyp_id) {
      desired_transition_prob *= std::dynamic_pointer_cast<BehaviorHypothesis>(behavior_hypothesis[hyp_id])
                            ->GetProbability(last_bark_action, last_mcts_state->get_observed_world(), front_agent_id);
    }
    current_state_prob = current_state_prob * desired_transition_prob;
    EXPECT_NEAR(current_state_prob, next_mcts_state->get_state_sequence_probability(), 0.00001);
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


int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();

}