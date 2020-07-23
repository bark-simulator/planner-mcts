// Copyright (c) 2020 Julian Bernhard
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "bark_mcts/python_wrapper/python_planner_uct.hpp"
#include "bark/python_wrapper/polymorphic_conversion.hpp"
#include <memory>
#include "bark_mcts/models/behavior/behavior_uct_single_agent.hpp"
//#include "bark/models/behavior/behavior_uct_single_agent_macro_actions.hpp"

#include "mcts/random_generator.h"

namespace py = pybind11;
using bark::commons::ParamsPtr;
using namespace bark::models::behavior;

void python_planner_uct(py::module m) {
  py::class_<BehaviorUCTSingleAgent, BehaviorModel,
             std::shared_ptr<BehaviorUCTSingleAgent>>(m,
                                                      "BehaviorUCTSingleAgent")
      .def(py::init<const bark::commons::ParamsPtr &>())
      .def("__repr__", [](const BehaviorUCTSingleAgent &m) {
        return "bark.behavior.BehaviorUCTSingleAgent";
      })
          .def(py::pickle(
      [](const BehaviorUCTSingleAgent& b) {
        return py::make_tuple(ParamsToPython(b.GetParams()));
      },
      [](py::tuple t) {
        if (t.size() != 1)
          throw std::runtime_error("Invalid behavior model state!");
        /* Create a new C++ instance */
        return new BehaviorUCTSingleAgent(PythonToParams(t[0].cast<py::tuple>()));
      }));
  
   py::class_<BehaviorUCTSingleAgentMacroActions, BehaviorModel,
             std::shared_ptr<BehaviorUCTSingleAgentMacroActions>>(
      m, "BehaviorUCTSingleAgentMacroActions")
      .def(py::init<const bark::commons::ParamsPtr &>())
      .def("__repr__", [](const BehaviorUCTSingleAgentMacroActions &m) {
        return "bark.behavior.BehaviorUCTSingleAgentMacroActions";
      })
          .def(py::pickle(
      [](const BehaviorUCTSingleAgentMacroActions& b) {
        return py::make_tuple(ParamsToPython(b.GetParams()));
      },
      [](py::tuple t) {
        if (t.size() != 1)
          throw std::runtime_error("Invalid behavior model state!");
        return new BehaviorUCTSingleAgentMacroActions(PythonToParams(t[0].cast<py::tuple>()));
      }));

}