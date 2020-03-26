// Copyright (c) 2019 fortiss GmbH
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "python/models/behavior.hpp"
#include "python_planner_uct.hpp"

namespace py = pybind11;

PYBIND11_MODULE(planner_uct, m) {
  python_behavior(m);
  python_planner_uct(m);
}