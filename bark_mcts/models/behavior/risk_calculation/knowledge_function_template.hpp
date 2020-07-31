// Copyright (c) 2020 Julian Bernhard
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.


#ifndef MODULES_MODELS_BEHAVIOR_RISK_CALCULATION_KNOWLEDGE_FUNCTION_TEMPLATE_HPP_
#define MODULES_MODELS_BEHAVIOR_RISK_CALCULATION_KNOWLEDGE_FUNCTION_TEMPLATE_HPP_

#include "bark_mcts/models/behavior/risk_calculation/common.hpp"
#include "bark/commons/distribution/distribution.hpp"

namespace bark {
namespace models {
namespace behavior {
namespace risk_calculation {


class  KnowledgeFunctionDefinition {
  public:
     KnowledgeFunctionDefinition(const RegionBoundaries& supporting_region) : 
        supporting_region_(supporting_region) {}

    virtual KnowledgeValue CalculateIntegral(const RegionBoundaries& integral_region) const = 0;

    typedef std::pair<bark::commons::Probability, RegionValue> KnowledgeSample;
    virtual KnowledgeSample Sample() const = 0;

    private:
      RegionBoundaries supporting_region_;
};

typedef std::shared_ptr< KnowledgeFunctionDefinition>  KnowledgeFunctionDefinitionPtr;

} // namespace risk calculation
} // namespace behavior
} // namespace models
} // namespace bark

#endif // MODULES_MODELS_BEHAVIOR_RISK_CALCULATION_KNOWLEDGE_FUNCTION_TEMPLATE_HPP_