// Copyright (c) 2020 Julian Bernhard
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.


#ifndef MODULES_MODELS_BEHAVIOR_RISK_CALCULATION_KNOWLEDGE_FUNCTION_TEMPLATE_HPP_
#define MODULES_MODELS_BEHAVIOR_RISK_CALCULATION_KNOWLEDGE_FUNCTION_TEMPLATE_HPP_

#include "bark_mcts/models/behavior/risk_calculation/prior_knowledge_region.hpp"
#include "bark/commons/distribution/distribution.hpp"

namespace bark {
namespace models {
namespace behavior {
namespace risk_calculation {


class KnowledgeFunctionDefinition : public bark::commons::BaseType {
  public:
     KnowledgeFunctionDefinition(const PriorKnowledgeRegion& supporting_region,
                                const bark::commons::ParamsPtr& params) :
                            bark::commons::BaseType(params),
                            supporting_region_(supporting_region) {}

    virtual KnowledgeValue CalculateIntegral(const RegionBoundaries& integral_region) const = 0;

    // First probability gives the probability of RegionValue with respect to full distribution
    // Second probability gives the probability of sample distribution (to correct using importance sampling)
    typedef std::tuple<RegionValue, bark::commons::Probability, bark::commons::Probability> KnowledgeSample;
    virtual KnowledgeSample Sample(const PriorKnowledgeRegion& sampling_region) const = 0;

    private:
      PriorKnowledgeRegion supporting_region_;
};

typedef std::shared_ptr<KnowledgeFunctionDefinition> KnowledgeFunctionDefinitionPtr;

} // namespace risk calculation
} // namespace behavior
} // namespace models
} // namespace bark

#endif // MODULES_MODELS_BEHAVIOR_RISK_CALCULATION_KNOWLEDGE_FUNCTION_TEMPLATE_HPP_