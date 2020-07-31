// Copyright (c) 2020 Julian Bernhard
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.


#ifndef MODULES_MODELS_BEHAVIOR_RISK_CALCULATION_PRIOR_KNOWLEDGE_FUNCTION_HPP_
#define MODULES_MODELS_BEHAVIOR_RISK_CALCULATION_PRIOR_KNOWLEDGE_FUNCTION_HPP_

#include "bark/commons/params/params.hpp"

#include "bark_mcts/models/behavior/risk_calculation/common.hpp"
#include "bark_mcts/models/behavior/risk_calculation/prior_knowledge_region.hpp"
#include "bark_mcts/models/behavior/risk_calculation/knowledge_function_template.hpp"
#include "bark_mcts/models/behavior/risk_calculation/scenario_risk_function.hpp"

namespace bark {
namespace models {
namespace behavior {
namespace risk_calculation {

class PriorKnowledgeFunction : public bark::commons::BaseType {
    public:
      PriorKnowledgeFunction(const PriorKnowledgeRegion& prior_knowledge_region,
                            const  KnowledgeFunctionDefinitionPtr& knowledge_function,
                            const bark::commons::ParamsPtr& params) : 
                            bark::commons::BaseType(params),
                            prior_knowledge_region_(prior_knowledge_region),
                            knowledge_function_(knowledge_function) {
        num_partitions_integration_=params->GetInt("PriorKnowledgeFunction::NumPartitionsIntegration", 
            "Specifies into how many cells the knowledge region is partitioned for integral calculation", 100);
        }

    KnowledgeValue GetIntegralKnowledeValue(const RegionBoundaries& knowledge_region) const;

    ScenarioRiskFunctionPtr CalculateScenarioRiskFunction(const ScenarioRiskFunctionTemplate& template_scenario_risk_function) const;

    PriorKnowledgeRegion GetPriorKnowledgeRegion() const { return prior_knowledge_region_; }

     KnowledgeFunctionDefinitionPtr GetKnowledgeFunction() const { return knowledge_function_; }

    private:
      unsigned int num_partitions_integration_;
      PriorKnowledgeRegion prior_knowledge_region_;
       KnowledgeFunctionDefinitionPtr knowledge_function_;
};


typedef std::shared_ptr<PriorKnowledgeFunction> PriorKnowledgeFunctionPtr;

} // namespace risk calculation
} // namespace behavior
} // namespace models
} // namespace bark

#endif // MODULES_MODELS_BEHAVIOR_RISK_CALCULATION_PRIOR_KNOWLEDGE_FUNCTION_HPP_