// Copyright (c) 2020 Julian Bernhard
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "bark_mcts/python_wrapper/polymorphic_conversion.hpp"
#include "bark/python_wrapper/polymorphic_conversion.hpp"

#include "bark_mcts/python_wrapper/python_risk_calculation.hpp"
#include "bark_mcts/models/behavior/risk_calculation/knowledge_function_definition/linear_knowledge_function_definition.hpp"
#include "bark_mcts/models/behavior/risk_calculation/prior_knowledge_function.hpp"
#include "bark_mcts/models/behavior/risk_calculation/prior_knowledge_region.hpp"
#include "bark_mcts/models/behavior/risk_calculation/scenario_risk_function.hpp"

namespace py = pybind11;
using bark::commons::ParamsPtr;
using namespace bark::models::behavior::risk_calculation;

void python_risk_calculation(py::module m) {

  py::class_<KnowledgeFunctionDefinition, PyKnowledgeFunctionDefinition, 
            bark::commons::BaseType,
             std::shared_ptr<KnowledgeFunctionDefinition>>(m,
    "KnowledgeFunctionDefinition")
    .def(py::init<const PriorKnowledgeRegion&,
        const bark::commons::ParamsPtr&>())
    .def("__repr__", [](const KnowledgeFunctionDefinition &m) {
      return "bark.behavior.KnowledgeFunctionDefinition";
    })
    .def("Sample", &KnowledgeFunctionDefinition::Sample)
    .def_property_readonly("supporting_region", &KnowledgeFunctionDefinition::GetSupportingRegion)
    .def("CalculateIntegral", &KnowledgeFunctionDefinition::CalculateIntegral);

   py::class_<LinearKnowledgeFunctionDefinition, KnowledgeFunctionDefinition,
             std::shared_ptr<LinearKnowledgeFunctionDefinition>>(m,
    "LinearKnowledgeFunctionDefinition")
    .def(py::init<const PriorKnowledgeRegion&,
        const bark::commons::ParamsPtr& >())
    .def("__repr__", [](const LinearKnowledgeFunctionDefinition &m) {
      return "bark.behavior.LinearKnowledgeFunctionDefinition";
    })
    .def(py::pickle(
      [](const LinearKnowledgeFunctionDefinition& pkf) {
        // We throw away other information such as last trajectories
        return py::make_tuple(pkf.GetSupportingRegion(), ParamsToPython(pkf.GetParams()));
      },
      [](py::tuple t) {
        if (t.size() != 2)
          throw std::runtime_error("Invalid LinearKnowledgeFunctionDefinition state!");
        return new LinearKnowledgeFunctionDefinition(t[0].cast<PriorKnowledgeRegion>(), PythonToParams(t[1].cast<py::tuple>()));
    }));

  py::class_<PriorKnowledgeRegion,
             std::shared_ptr<PriorKnowledgeRegion>>(m,
    "PriorKnowledgeRegion")
    .def(py::init<const RegionBoundaries&>())
    .def("__repr__", [](const PriorKnowledgeRegion &m) {
      return "bark.behavior.PriorKnowledgeRegion";
    })
    .def_property_readonly("definition", &PriorKnowledgeRegion::GetDefinition)
    .def("Partition", &PriorKnowledgeRegion::Partition)
    .def(py::pickle(
      [](const PriorKnowledgeRegion& pkr) {
        // We throw away other information such as last trajectories
        return py::make_tuple(pkr.GetDefinition());
      },
      [](py::tuple t) {
        if (t.size() != 1)
          throw std::runtime_error("Invalid PriorKnowledgeRegion state!");
        return new PriorKnowledgeRegion(t[0].cast<RegionBoundaries>());
      }));

  py::class_<PriorKnowledgeFunction,
             std::shared_ptr<PriorKnowledgeFunction>>(m,
    "PriorKnowledgeFunction")
    .def(py::init<const PriorKnowledgeRegion&, 
             const KnowledgeFunctionDefinitionPtr&,
              const bark::commons::ParamsPtr&>())
    .def("__repr__", [](const PriorKnowledgeFunction &m) {
      return "bark.behavior.PriorKnowledgeFunction";
    })
    .def("CalculateScenarioRiskFunction", &PriorKnowledgeFunction::CalculateScenarioRiskFunction)
    .def_property_readonly("knowledge_function_definition", &PriorKnowledgeFunction::GetKnowledgeFunction)
    .def(py::pickle(
      [](const PriorKnowledgeFunction& pkf) {
        // We throw away other information such as last trajectories
        return py::make_tuple(pkf.GetPriorKnowledgeRegion(),
                            pkf.GetKnowledgeFunction(),
                            ParamsToPython(pkf.GetParams()));
      },
      [](py::tuple t) {
        if (t.size() != 3)
          throw std::runtime_error("Invalid PriorKnowledgeFunction state!");
        return new PriorKnowledgeFunction(t[0].cast<PriorKnowledgeRegion>(),
                                      t[1].cast<KnowledgeFunctionDefinitionPtr>(),
                                      PythonToParams(t[2].cast<py::tuple>()));
      }));

  py::class_<ScenarioRiskFunction,
             std::shared_ptr<ScenarioRiskFunction>>(m,
    "ScenarioRiskFunction")
    .def(py::init<const KnowledgeFunctionDefinitionPtr&, 
             const double&>())
    .def("__repr__", [](const ScenarioRiskFunction &m) {
      return "bark.behavior.ScenarioRiskFunction";
    })
    .def_property_readonly("normalization_constant", &ScenarioRiskFunction::GetNormalizationConstant)
    .def_property_readonly("scenario_risk_function_definition", &ScenarioRiskFunction::GetRiskFunctionDefinition)
    .def("CalculateIntegralValue", &ScenarioRiskFunction::CalculateIntegralValue)
    .def(py::pickle(
      [](const ScenarioRiskFunction& srf) {
        // We throw away other information such as last trajectories
        return py::make_tuple(KnowledgeFunctionDefinitionToPython(srf.GetRiskFunctionDefinition()),
                            srf.GetNormalizationConstant());
      },
      [](py::tuple t) {
        if (t.size() != 2)
          throw std::runtime_error("Invalid ScenarioRiskFunction state!");
        return new ScenarioRiskFunction(PythonToKnowledgeFunctionDefinition(t[0].cast<py::tuple>()),
                                      t[1].cast<double>());
      }));

  m.def("CalculateRegionBoundariesArea", &CalculateRegionBoundariesArea);
}