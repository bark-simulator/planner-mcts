// Copyright (c) 2020 Julian Bernhard
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.


#ifndef MODULES_MODELS_BEHAVIOR_RISK_CALCULATION_PRIOR_KNOWLEDGE_FUNCTION_HPP_
#define MODULES_MODELS_BEHAVIOR_RISK_CALCULATION_PRIOR_KNOWLEDGE_FUNCTION_HPP_

#include <functional>

#include "bark/commons/params/params.hpp"

#include "bark_mcts/models/behavior/risk_calculation/prior_knowledge_region.hpp"
#include "bark_mcts/models/behavior/risk_calculation/scenario_risk_function.hpp"


namespace bark {
namespace models {
namespace behavior {
namespace risk_calculation {

class PriorKnowledgeFunction : public bark::commons::BaseType {
    public:
      PriorKnowledgeFunction(PriorKnowledgeRegion &prior_knowledge_region,
                            const KnowledgeFunction& knowledge_function_,
                            const bark::commons::ParamsPtr& params) : BaseType(params) {
        num_partitions_integration_=params->GetInt("PriorKnowledgeFunction::NumPartitionsIntegration", 
            "Specifies into how many cells the knowledge region is partitioned for integral calculation", 1000);
        }

    KnowledgeValue GetKnowledeValue(const RegionValue& value) const;
    KnowledgeValue GetIntegralKnowledeValue(const KnowledgeRegion& knowledge_region) const;

    ScenarioRiskFunction CalculateScenarioRiskFunction(const KnowledgeFunction& template_function_scenario_risk) const;

    private:
      unsigned int num_partitions_integration_;
      PriorKnowledgeRegion knowledge_region_;
      KnowledgeFunction knowledge_function;
}


typdef std::shared_ptr<PriorKnowledgeFunction> PriorKnowledgeFunctionPtr;

} // namespace risk calculation
} // namespace behavior
} // namespace models
} // namespace bark

#endif // MODULES_MODELS_BEHAVIOR_RISK_CALCULATION_PRIOR_KNOWLEDGE_FUNCTION_HPP_