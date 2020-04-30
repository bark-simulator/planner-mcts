

// Copyright (c) 2019 fortiss GmbH
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "modules/models/behavior/tests/test_helpers.hpp"
#include <vector>
#include "modules/commons/params/default_params.hpp"
#include "modules/geometry/commons.hpp"
#include "modules/geometry/line.hpp"
#include "modules/geometry/polygon.hpp"
#include "modules/models/behavior/idm/stochastic/idm_stochastic.hpp"
#include "modules/world/goal_definition/goal_definition_state_limits_frenet.hpp"
#include "modules/models/dynamic/single_track.hpp"
#include "modules/models/execution/interpolation/interpolate.hpp"
#include "modules/world/observed_world.hpp"
#include "modules/world/tests/make_test_xodr_map.hpp"

using namespace modules::models::dynamic;
using namespace modules::models::execution;
using namespace modules::commons;
using namespace modules::models::behavior;
using namespace modules::world::map;
using namespace modules::models::dynamic;
using namespace modules::world::goal_definition;
using namespace modules::world::tests;
using namespace modules::geometry;

using modules::geometry::standard_shapes::CarRectangle;
using modules::world::World;
using modules::world::WorldPtr;
using modules::world::ObservedWorldPtr;
using modules::world::goal_definition::GoalDefinitionStateLimitsFrenet;
using modules::world::goal_definition::GoalDefinitionPtr;
using modules::world::map::MapInterface;
using modules::world::map::MapInterfacePtr;
using modules::world::objects::Agent;
using modules::world::objects::AgentPtr;
using modules::world::opendrive::OpenDriveMapPtr;
using modules::world::tests::MakeXodrMapOneRoadTwoLanes;

constexpr float pos_y_right_lane = -4.75;
constexpr float pos_y_left_lane =  -1.75;

AgentPtr modules::world::tests::CreateAgent(bool right_lane, float s, float velocity, bool goal_right_lane,
                         const ParamsPtr& params, const MapInterfacePtr& map_interface) {
    ExecutionModelPtr exec_model(new ExecutionModelInterpolate(params));
  DynamicModelPtr dyn_model(new SingleTrackModel(params));
  auto beh_model_idm_stoch = std::make_shared<BehaviorIDMStochastic>(params);
  auto shape = CarRectangle();

  float pos_y = pos_y_left_lane;
  if(right_lane) {
      pos_y = pos_y_right_lane;
  }

  Polygon polygon(Pose(0, 0, 0), std::vector<Point2d>{Point2d(-10, 0.2), Point2d(10, 0.2), Point2d(10, -0.2), Point2d(-10, -0.2), Point2d(-10, 0.2)});
  std::shared_ptr<Polygon> goal_polygon(std::dynamic_pointer_cast<Polygon>(polygon.Translate(Point2d(50, pos_y)))); // < move the goal polygon into the driving corridor in front of the ego vehicle
  auto road_corridor = map_interface->GenerateRoadCorridor(Point2d(0.0,0.0), *goal_polygon);

  float goal_pos_y = pos_y_left_lane;
  if(goal_right_lane) {
      goal_pos_y = pos_y_right_lane;
  }
  auto goal_lane_corr = road_corridor->GetCurrentLaneCorridor(Point2d(5.0, goal_pos_y));
  auto goal_definition = std::make_shared<GoalDefinitionStateLimitsFrenet>(
                goal_lane_corr->GetCenterLine(), std::pair<float,float>{0.2, 0.2},
                     std::pair<float,float>{0.08, 0.08}, std::pair<float,float>{10, 20});

  State init_state(static_cast<int>(StateDefinition::MIN_STATE_SIZE));
  init_state << 0.0, s, pos_y, 0.0, velocity;
  return std::make_shared<Agent>(init_state, beh_model_idm_stoch, dyn_model, exec_model,
                            shape, params, goal_definition,
                            map_interface, modules::geometry::Model3D());
}

