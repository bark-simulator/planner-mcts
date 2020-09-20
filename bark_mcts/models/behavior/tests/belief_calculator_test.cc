// Copyright (c) 2020 Julian Bernhard
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.
#include <chrono>
#include "gtest/gtest.h"
#include "bark_mcts/models/behavior/belief_calculator/belief_calculator.hpp"
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
using bark::models::dynamic::Trajectory;
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


TEST(belief_calculator, two_hypothesis) {
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

  bark::models::behavior::BeliefCalculator belief_calculator(params, behavior_hypothesis);

  belief_calculator.BeliefUpdate(observed_world);
  auto beliefs = belief_calculator.GetBeliefs();
}


int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();

}