// Copyright (c) 2020 Julian Bernhard
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "bark/python_wrapper/models/behavior.hpp"
#include "bark/python_wrapper/python_planner_uct.hpp"

namespace py = pybind11;

PYBIND11_MODULE(planner_uct, m) {
  python_behavior(m);
  python_planner_uct(m);
}