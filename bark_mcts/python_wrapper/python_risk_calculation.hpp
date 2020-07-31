// Copyright (c) 2020 Julian Bernhard
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef PYTHON_PYTHON_RISK_CALCULATION_HPP_
#define PYTHON_PYTHON_RISK_CALCULATION_HPP_

#include <pybind11/functional.h>
#include "bark/python_wrapper/common.hpp"
#include "bark_mcts/models/behavior/risk_calculation/knowledge_function_template.hpp"

namespace py = pybind11;

using bark::models::behavior::risk_calculation::PriorKnowledgeFunctionDefinition;
using bark::models::behavior::risk_calculation::RegionValue;
using bark::models::behavior::risk_calculation::KnowledgeValue;
using bark::models::behavior::risk_calculation::RegionBoundaries;
using bark::commons::Probability;

class PyPriorKnowledgeFunctionDefinition : public  PriorKnowledgeFunctionDefinition {
 public:
  using PriorKnowledgeFunctionDefinition:: PriorKnowledgeFunctionDefinition;

  KnowledgeValue CalculateIntegral(const RegionBoundaries& integral_region) const {
    PYBIND11_OVERLOAD_PURE(KnowledgeValue,  PriorKnowledgeFunctionDefinition,
                           CalculateIntegral, integral_region);
  }

  PriorKnowledgeFunctionDefinition::KnowledgeSample Sample() const {
    PYBIND11_OVERLOAD_PURE(PriorKnowledgeFunctionDefinition::KnowledgeSample,  PriorKnowledgeFunctionDefinition,
                           Sample);
  }

};


void python_risk_calculation(py::module m);

#endif  // PYTHON_PYTHON_RISK_CALCULATION_HPP_