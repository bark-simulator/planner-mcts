// Copyright (c) 2019 fortiss GmbH
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "src/behavior_uct_single_agent.hpp"
#include "mcts/random_generator.h"
#include "python/python_planner_uct.hpp"

namespace py = pybind11;
using modules::models::behavior::BehaviorUCTSingleAgent;
using modules::models::behavior::BehaviorModel;
using modules::commons::Params;

std::mt19937 mcts::RandomGenerator::random_generator_;

void python_planner_uct(py::module m)
{
    py::class_<BehaviorUCTSingleAgent,
             BehaviorModel,
             std::shared_ptr<BehaviorUCTSingleAgent>>(m, "BehaviorUCTSingleAgent")
      .def(py::init<modules::commons::Params *>())
      .def("__repr__", [](const BehaviorUCTSingleAgent &m) {
        return "bark.behavior.BehaviorUCTSingleAgent";
      })
      .def(py::pickle(
        [](const BehaviorUCTSingleAgent &b) { 
            return py::make_tuple(b.get_last_trajectory()); // 0
        },
        [](py::tuple t) { // __setstate__
            if (t.size() != 1)
                throw std::runtime_error("Invalid behavior model state!");

            /* Create a new C++ instance */
            return new BehaviorUCTSingleAgent(nullptr); // param pointer must be set afterwards
        }));
}