// Copyright (c) 2020 Julian Bernhard
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef BARK_MCTS_PYTHON_WRAPPER_POLYMORPHIC_CONVERSION_HPP_
#define BARK_MCTS_PYTHON_WRAPPER_POLYMORPHIC_CONVERSION_HPP_

#include "bark/python_wrapper/common.hpp"
#include "bark_mcts/models/behavior/risk_calculation/knowledge_function_definition/knowledge_function_definition.hpp"

namespace py = pybind11;
using bark::commons::ParamsPtr;
using bark::models::behavior::risk_calculation::KnowledgeFunctionDefinitionPtr;

// For pickle we need conversion functions between the genereric base types and
// the derived types

py::tuple KnowledgeFunctionDefinitionToPython(KnowledgeFunctionDefinitionPtr knowledge_function_definition);
KnowledgeFunctionDefinitionPtr PythonToKnowledgeFunctionDefinition(py::tuple t);

#endif  // BARK_MCTS_PYTHON_WRAPPER_POLYMORPHIC_CONVERSION_HPP_
