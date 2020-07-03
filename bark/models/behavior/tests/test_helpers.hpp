// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef TESTS_TEST_HELPERS_HPP_
#define TESTS_TEST_HELPERS_HPP_

#include "bark/geometry/commons.hpp"
#include "bark/world/goal_definition/goal_definition.hpp"
#include "bark/world/goal_definition/goal_definition_polygon.hpp"
#include "bark/world/map/map_interface.hpp"
#include "bark/world/observed_world.hpp"

namespace bark {
namespace world {
namespace tests {

using bark::commons::ParamsPtr;
using bark::world::map::MapInterfacePtr;
using bark::world::AgentPtr;

AgentPtr CreateAgent(bool right_lane, float s, float velocity, bool goal_right_lane,
                         const ParamsPtr& params, const MapInterfacePtr& map_interface);

}  // namespace tests
}  // namespace world
}  // namespace bark

#endif  // TESTS_TEST_HELPERS_HPP_