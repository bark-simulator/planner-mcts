// Copyright (c) 2019 fortiss GmbH
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "python/python_planner_uct.hpp"
#include "python/polymorphic_conversion.hpp"
#include <memory>
#include "src/behavior_uct_single_agent.hpp"
#include "src/behavior_uct_single_agent_macro_actions.hpp"
#include "src/behav_macro_actions_from_param_server.hpp"
#include "src/behavior_uct_hypothesis.hpp"

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
       const BehaviorMotionPrimitivesPtr&,
       const std::vector<BehaviorHypothesisPtr>&>())
      .def("__repr__", [](const BehaviorUCTHypothesis &m) {
        return "bark.behavior.BehaviorUCTSingleAgentMacroActions";
      })
      .def(py::pickle(
      [](const BehaviorUCTHypothesis& b) {
        py::list list;
        auto hypotheses = b.GetBehaviorHypotheses();
        for (const auto& hypothesis : hypotheses) {
          list.append(BehaviorModelToPython(hypothesis));
        }
        return py::make_tuple(ParamsToPython(b.GetParams()),
               BehaviorModelToPython(b.GetEgoBehavior()), list);
      },
      [](py::tuple t) {
        if (t.size() != 3)
          throw std::runtime_error("Invalid behavior model state!");
        /* Create a new C++ instance */
        std::vector<BehaviorHypothesisPtr> hypotheses;
        const auto& list =  t[2].cast<py::list>();
        for (const auto& el : list) {
          auto behavior_model = PythonToBehaviorModel(el.cast<py::tuple>());
          auto hypothesis_ptr = std::dynamic_pointer_cast<BehaviorHypothesis>(behavior_model);
          if(!hypothesis_ptr) {
            throw std::runtime_error("Could not cast behavior model to hypothesis model!");
          }
          hypotheses.push_back(hypothesis_ptr);
        }
        const auto ego_behavior_model = PythonToBehaviorModel(t[1].cast<py::tuple>());
        auto ego_behavior_motion_primitive = std::dynamic_pointer_cast<BehaviorMotionPrimitives>(
                ego_behavior_model);
        if(!ego_behavior_motion_primitive) {
          throw std::runtime_error("Could not cast behavior model to motion primitive model!");
        }
        return new BehaviorUCTHypothesis(PythonToParams(t[0].cast<py::tuple>()),
                ego_behavior_motion_primitive,
                hypotheses);
      }));

  m.def("BehaviorMacroActionFromParamServer", &BehaviorMacroActionsFromParamServer);
}