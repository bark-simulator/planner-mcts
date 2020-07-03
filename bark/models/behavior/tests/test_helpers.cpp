

// Copyright (c) 2020 Julian Bernhard
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "bark/models/behavior/tests/test_helpers.hpp"
#include <vector>
#include "bark/commons/params/default_params.hpp"
#include "bark/geometry/commons.hpp"
#include "bark/geometry/line.hpp"
#include "bark/geometry/polygon.hpp"
#include "bark/models/behavior/idm/stochastic/idm_stochastic.hpp"
#include "bark/world/goal_definition/goal_definition_state_limits_frenet.hpp"
#include "bark/models/dynamic/single_track.hpp"
#include "bark/models/execution/interpolation/interpolate.hpp"
#include "bark/world/observed_world.hpp"
#include "bark/world/tests/make_test_xodr_map.hpp"

using namespace bark::models::dynamic;
using namespace bark::models::execution;
using namespace bark::commons;
using namespace bark::models::behavior;
using namespace bark::world::map;
using namespace bark::models::dynamic;
using namespace bark::world::goal_definition;
using namespace bark::world::tests;
using namespace bark::geometry;

using bark::geometry::standard_shapes::CarRectangle;
using bark::world::World;
using bark::world::WorldPtr;
using bark::world::ObservedWorldPtr;
using bark::world::goal_definition::GoalDefinitionStateLimitsFrenet;
using bark::world::goal_definition::GoalDefinitionPtr;
using bark::world::map::MapInterface;
using bark::world::map::MapInterfacePtr;
using bark::world::objects::Agent;
using bark::world::objects::AgentPtr;
using bark::world::opendrive::OpenDriveMapPtr;
using bark::world::tests::MakeXodrMapOneRoadTwoLanes;

constexpr float pos_y_right_lane = -4.75;
constexpr float pos_y_left_lane =  -1.75;

AgentPtr bark::world::tests::CreateAgent(bool right_lane, float s, float velocity, bool goal_right_lane,
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
                            map_interface, bark::geometry::Model3D());
}

