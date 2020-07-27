// Copyright (c) 2020 Julian Bernhard
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.


#ifndef MODULES_MODELS_BEHAVIOR_RISK_CALCULATION_PRIOR_KNOWLEDGE_FUNCTION_HPP_
#define MODULES_MODELS_BEHAVIOR_RISK_CALCULATION_PRIOR_KNOWLEDGE_FUNCTION_HPP_

namespace bark {
namespace models {
namespace behavior {
namespace prior_knowledge {

typedef std::function<KnowledgeValue(std::unordered_map<DimensionName, KnowledgeRegion>)> KnowledgeFunction;

class PriorKnowledgeFunction : public BaseType {
    public:
     PriorKnowledgeFunction(PriorKnowledgeRegion &prior_knowledge_region,
                            const KnowledgeFunction& knowledge_function_,
                            const ParamsPtr& params) : BaseType(params) {
        num_partitions_integration_=params->GetInt("PriorKnowledgeFunction::NumPartitionsIntegration", 
            "Specifies into how many cells the knowledge region is partitioned for integral calculation", 1000);
    }

    KnowledgeValue GetKnowledeValue(const& RegionValue) const;
    KnowledgeValue GetIntegralKnowledeValue(const& KnowledgeRegion) const;

    ScenarioRiskFunction CalculateScenarioRiskFunction(const KnowledgeFunction& template_function_scenario_risk) const {
        // idea we give a template function as lambda with fixed parameters depending on
        // the region value, e.g (1*x +2) or (x + 0.1x*x), the normalization scaling c is then calcuated that
        // the integral gets 1
        double integration_sum = 0.0f;
        for(const auto&region : prior_knowledge_function_->GetKnowledgeRegion()->Partition(num_partitions_integration_)) {
            auto prior_knowledge_value = prior_knowledge_function_->GetIntegralKnowledeValue()
            auto template_scenario_integral_value = template_function_scenario_risk(region);
            sum += prior_knowledge_value*template_scenario_integral_value;
        }

        double normalization_factor = 1/sum;
        return ScenarioRiskFunction()
    }
    private:
    unsigned int num_partitions_integration_;
    KnowledgeRegion knowledge_region_;
    KnowledgeFunction knowledge_function;
}

typdef std::shared_ptr<PriorKnowledgeFunction> PriorKnowledgeFunctionPtr;

} // namespace risk calculation
} // namespace behavior
} // namespace models
} // namespace bark

#endif // MODULES_MODELS_BEHAVIOR_RISK_CALCULATION_PRIOR_KNOWLEDGE_FUNCTION_HPP_