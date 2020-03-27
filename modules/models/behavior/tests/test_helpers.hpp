// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef TESTS_TEST_HELPERS_HPP_
#define TESTS_TEST_HELPERS_HPP_

#include "modules/geometry/commons.hpp"
#include "modules/world/goal_definition/goal_definition.hpp"
#include "modules/world/goal_definition/goal_definition_polygon.hpp"
#include "modules/world/map/map_interface.hpp"
#include "modules/world/observed_world.hpp"

namespace modules {
namespace world {
namespace tests {

using modules::commons::ParamsPtr;
using modules::world::map::MapInterfacePtr;
using modules::world::AgentPtr;

AgentPtr CreateAgent(bool right_lane, float s, float velocity, bool goal_right_lane,
                         const ParamsPtr& params, const MapInterfacePtr& map_interface);

}  // namespace tests
}  // namespace world
}  // namespace modules

#endif  // TESTS_TEST_HELPERS_HPP_