// Copyright (c) 2019 fortiss GmbH
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "python/python_planner_uct.hpp"
#include <memory>
#include "src/behavior_uct_single_agent.hpp"
#include "src/behavior_uct_single_agent_macro_actions.hpp"

#include "mcts/random_generator.h"

namespace py = pybind11;
using modules::commons::ParamsPtr;
using modules::models::behavior::BehaviorModel;
using modules::models::behavior::BehaviorUCTSingleAgent;
using modules::models::behavior::BehaviorUCTSingleAgentMacroActions;

void python_planner_uct(py::module m) {
  py::class_<BehaviorUCTSingleAgent, BehaviorModel,
             std::shared_ptr<BehaviorUCTSingleAgent>>(m,
                                                      "BehaviorUCTSingleAgent")
      .def(py::init<const modules::commons::ParamsPtr &>())
      .def("__repr__", [](const BehaviorUCTSingleAgent &m) {
        return "bark.behavior.BehaviorUCTSingleAgent";
      });
  py::class_<BehaviorUCTSingleAgentMacroActions, BehaviorModel,
             std::shared_ptr<BehaviorUCTSingleAgentMacroActions>>(
      m, "BehaviorUCTSingleAgentMacroActions")
      .def(py::init<const modules::commons::ParamsPtr &>())
      .def("__repr__", [](const BehaviorUCTSingleAgentMacroActions &m) {
        return "bark.behavior.BehaviorUCTSingleAgentMacroActions";
      });
}