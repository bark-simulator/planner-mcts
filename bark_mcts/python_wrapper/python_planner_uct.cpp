// Copyright (c) 2020 Julian Bernhard
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "bark_mcts/python_wrapper/python_planner_uct.hpp"
#include "bark_mcts/python_wrapper/python_risk_calculation.hpp"
#include "bark_mcts/python_wrapper/polymorphic_conversion.hpp"
#include "bark/python_wrapper/polymorphic_conversion.hpp"
#include <memory>
#include "bark_mcts/models/behavior/behavior_uct_single_agent.hpp"
#include "bark_mcts/models/behavior/belief_calculator/belief_calculator.hpp"
#include "bark_mcts/models/behavior/behavior_uct_hypothesis.hpp"
#include "bark_mcts/models/behavior/behavior_uct_cooperative.hpp"
#include "bark_mcts/models/behavior/behavior_uct_risk_constraint.hpp"
#include "bark_mcts/models/behavior/hypothesis/idm/hypothesis_idm.hpp"

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
  
/*  py::class_<BehaviorUCTSingleAgentMacroActions, BehaviorModel,
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
      })); */

 py::class_<BehaviorUCTHypothesis, BehaviorModel,
             std::shared_ptr<BehaviorUCTHypothesis>>(
      m, "BehaviorUCTHypothesis")
      .def(py::init<const bark::commons::ParamsPtr &,
       const std::vector<BehaviorModelPtr>&>())
      .def("__repr__", [](const BehaviorUCTHypothesis &m) {
        return "bark.behavior.BehaviorUCTHypothesis";
      })
      .def_property_readonly("hypotheses", &BehaviorUCTHypothesis::GetBehaviorHypotheses)
      .def_property_readonly("last_extracted_mcts_edges", &BehaviorUCTHypothesis::GetLastMctsEdgeInfo)
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
        std::vector<BehaviorModelPtr> hypotheses;
        const auto& list =  t[1].cast<py::list>();
        for (const auto& el : list) {
          hypotheses.push_back(PythonToBehaviorModel(el.cast<py::tuple>()));
        }
        return new BehaviorUCTHypothesis(PythonToParams(t[0].cast<py::tuple>()),
                hypotheses);
      }));

   py::class_<BehaviorUCTCooperative, BehaviorModel,
             std::shared_ptr<BehaviorUCTCooperative>>(
      m, "BehaviorUCTCooperative")
      .def(py::init<const bark::commons::ParamsPtr &>())
      .def("__repr__", [](const BehaviorUCTCooperative &m) {
        return "bark.behavior.BehaviorUCTCooperative";
      })
      .def_property_readonly("last_extracted_mcts_edges", &BehaviorUCTCooperative::GetLastMctsEdgeInfo)
      .def(py::pickle(
      [](const BehaviorUCTCooperative& b) {
        return py::make_tuple(ParamsToPython(b.GetParams()));
      },
      [](py::tuple t) {
        if (t.size() != 1)
          throw std::runtime_error("Invalid behavior model state!");
        return new BehaviorUCTCooperative(PythonToParams(t[0].cast<py::tuple>()));
      }));

   py::class_<BehaviorUCTRiskConstraint, BehaviorModel,
             std::shared_ptr<BehaviorUCTRiskConstraint>>(
      m, "BehaviorUCTRiskConstraint")
      .def(py::init<const bark::commons::ParamsPtr &,
       const std::vector<BehaviorModelPtr>&,
       const risk_calculation::ScenarioRiskFunctionPtr&>())
      .def("__repr__", [](const BehaviorUCTRiskConstraint &m) {
        return "bark.behavior.BehaviorUCTRiskConstraint";
      })
      .def_property_readonly("hypotheses", &BehaviorUCTRiskConstraint::GetBehaviorHypotheses)
      .def_property_readonly("last_extracted_mcts_edges", &BehaviorUCTRiskConstraint::GetLastMctsEdgeInfo)
      .def_property_readonly("scenario_risk_function", &BehaviorUCTRiskConstraint::GetScenarioRiskFunction)
      .def(py::pickle(
      [](const BehaviorUCTRiskConstraint& b) -> py::tuple {
        py::list list;
        auto hypotheses = b.GetBehaviorHypotheses();
        for (const auto& hypothesis : hypotheses) {
          list.append(BehaviorModelToPython(hypothesis));
        }
        if ( b.GetScenarioRiskFunction()) {
          return py::make_tuple(ParamsToPython(b.GetParams()), list, b.GetScenarioRiskFunction());
        } else {
          return py::make_tuple(ParamsToPython(b.GetParams()), list);
        }
      },
      [](py::tuple t) {
        if (t.size() != 2 && t.size() != 3)
          throw std::runtime_error("Invalid behavior model state!");
        std::vector<BehaviorModelPtr> hypotheses;
        const auto& list =  t[1].cast<py::list>();
        for (const auto& el : list) {
          hypotheses.push_back(PythonToBehaviorModel(el.cast<py::tuple>()));
        }
        if (t.size() == 3) {
          return new BehaviorUCTRiskConstraint(PythonToParams(t[0].cast<py::tuple>()),
                hypotheses, std::make_shared<risk_calculation::ScenarioRiskFunction>(
                                    t[2].cast<risk_calculation::ScenarioRiskFunction>()));
        } else {
          return new BehaviorUCTRiskConstraint(PythonToParams(t[0].cast<py::tuple>()),
                hypotheses, nullptr);
        }
      }));

  py::class_<BehaviorHypothesis,
             BehaviorModel,
             std::shared_ptr<BehaviorHypothesis>>(m,
    "BehaviorHypothesis");
    
      py::class_<BehaviorHypothesisIDM,
             BehaviorHypothesis,
             std::shared_ptr<BehaviorHypothesisIDM>>(m, "BehaviorHypothesisIDM", py::multiple_inheritance())
    .def(py::init<const bark::commons::ParamsPtr&>())
    .def("__repr__", [](const BehaviorHypothesisIDM &m) {
      return "bark.behavior.BehaviorHypothesisIDM";
    })
    .def(py::pickle(
      [](const BehaviorHypothesisIDM& b) {
        // We throw away other information such as last trajectories
        return py::make_tuple(ParamsToPython(b.GetParams()));
      },
      [](py::tuple t) {
        if (t.size() != 1)
          throw std::runtime_error("Invalid behavior model state!");
        return new BehaviorHypothesisIDM(PythonToParams(t[0].cast<py::tuple>()));
      }));

    py::class_<BeliefCalculator,
             std::shared_ptr<BeliefCalculator>>(m,
    "BeliefCalculator")
    .def(py::init<const bark::commons::ParamsPtr&,
              const std::vector<BehaviorModelPtr>&>())
    .def("BeliefUpdate", &BeliefCalculator::BeliefUpdate)
    .def("GetBeliefs", &BeliefCalculator::GetBeliefs)
    .def(py::pickle(
      [](const BeliefCalculator& bc) {
        py::list list;
        auto hypotheses = bc.GetBehaviorHypotheses();
        for (const auto& hypothesis : hypotheses) {
          list.append(BehaviorModelToPython(hypothesis));
        }
        return py::make_tuple(ParamsToPython(bc.GetParams()), list);
      },
      [](py::tuple t) {
        if (t.size() != 2)
          throw std::runtime_error("Invalid belief calculator state!");
        std::vector<BehaviorModelPtr> hypotheses;
        const auto& list =  t[1].cast<py::list>();
        for (const auto& el : list) {
          hypotheses.push_back(PythonToBehaviorModel(el.cast<py::tuple>()));
        }        
        return new BeliefCalculator(PythonToParams(t[0].cast<py::tuple>()), hypotheses);
      }));
      python_risk_calculation(m.def_submodule("risk_calculation"));

}