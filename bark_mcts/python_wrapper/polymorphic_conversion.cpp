// Copyright (c) 2020 Julian Bernhard
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "bark_mcts/python_wrapper/polymorphic_conversion.hpp"
#include "bark_mcts/models/behavior/risk_calculation/knowledge_function_definition/linear_knowledge_function_definition.hpp"
#include "bark/commons/params/setter_params.hpp"

using bark::models::behavior::risk_calculation::LinearKnowledgeFunctionDefinition;
namespace py = pybind11;

py::tuple KnowledgeFunctionDefinitionToPython(KnowledgeFunctionDefinitionPtr knowledge_function_definition) {
  std::string knowledge_function_definition_name;
  if (typeid(*knowledge_function_definition) == typeid(LinearKnowledgeFunctionDefinition)) {
    knowledge_function_definition_name = "LinearKnowledgeFunctionDefinition";
  } else {
    LOG(FATAL) << "Unknown KnowledeFunctioDefinition for polymorphic conversion to python: "
               << typeid(*knowledge_function_definition).name();
  }
  return py::make_tuple(knowledge_function_definition, knowledge_function_definition_name);
}

KnowledgeFunctionDefinitionPtr PythonToKnowledgeFunctionDefinition(py::tuple t) {
  std::string knowledge_function_definition_name = t[1].cast<std::string>();
  if (knowledge_function_definition_name.compare("LinearKnowledgeFunctionDefinition") == 0) {
    return std::make_shared<LinearKnowledgeFunctionDefinition>(
        t[0].cast<LinearKnowledgeFunctionDefinition>());
  } else {
       LOG(FATAL) << "Unknown KnowledeFunctioDefinition for polymorphic conversion to cpp: "
               << knowledge_function_definition_name;
  }
}