// Copyright (c) 2019 fortiss GmbH
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "python/python_planner_uct.hpp"
#include "python/polymorphic_conversion.hpp"
#include <memory>
#include "modules/models/behavior/behavior_uct_single_agent.hpp"
#include "modules/models/behavior/behavior_uct_single_agent_macro_actions.hpp"
#include "modules/models/behavior/behavior_uct_hypothesis.hpp"

#include "mcts/random_generator.h"

namespace py = pybind11;
using modules::commons::ParamsPtr;
using namespace modules::models::behavior;

void python_planner_uct(py::module m) {
  py::class_<BehaviorUCTSingleAgent, BehaviorModel,
             std::shared_ptr<BehaviorUCTSingleAgent>>(m,
                                                      "BehaviorUCTSingleAgent")
      .def(py::init<const modules::commons::ParamsPtr &>())
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
      .def(py::init<const modules::commons::ParamsPtr &>())
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
        /* Create a new C++ instance */
        return new BehaviorUCTSingleAgentMacroActions(PythonToParams(t[0].cast<py::tuple>()));
      }));

  py::class_<BehaviorUCTHypothesis, BehaviorModel,
             std::shared_ptr<BehaviorUCTHypothesis>>(
      m, "BehaviorUCTHypothesis")
      .def(py::init<const modules::commons::ParamsPtr &,
       const std::vector<BehaviorModelPtr>&>())
      .def("__repr__", [](const BehaviorUCTHypothesis &m) {
        return "bark.behavior.BehaviorUCTHypothesis";
      })
      .def_property_readonly("hypotheses", &BehaviorUCTHypothesis::GetBehaviorHypotheses)
      .def(py::pickle(
      [](const BehaviorUCTHypothesis& b) {
        py::list list;
        auto hypotheses = b.GetBehaviorHypotheses();
        for (const auto& hypothesis : hypotheses) {
          list.append(BehaviorModelToPython(hypothesis));
        }
        return py::make_tuple(ParamsToPython(b.GetParams()), list);
      },
      [](py::tuple t) {
        if (t.size() != 2)
          throw std::runtime_error("Invalid behavior model state!");
        /* Create a new C++ instance */
        std::vector<BehaviorModelPtr> hypotheses;
        const auto& list =  t[1].cast<py::list>();
        for (const auto& el : list) {
          hypotheses.push_back(PythonToBehaviorModel(el.cast<py::tuple>()));
        }
        return new BehaviorUCTHypothesis(PythonToParams(t[0].cast<py::tuple>()),
                hypotheses);
      }));

}