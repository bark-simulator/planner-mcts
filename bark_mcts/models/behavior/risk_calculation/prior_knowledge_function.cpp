// Copyright (c) 2020 Julian Bernhard
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "bark_mcts/models/behavior/risk_calculation/prior_knowledge_function.hpp"

using namespace bark::models::behavior::risk_calculation;

KnowledgeValue PriorKnowledgeFunction::GetKnowledeValue(const RegionValue& value) const {

}

KnowledgeValue PriorKnowledgeFunction::GetIntegralKnowledeValue(const KnowledgeRegion& knowledge_region) const {

}

ScenarioRiskFunction PriorKnowledgeFunction::CalculateScenarioRiskFunction(const KnowledgeFunction& template_function_scenario_risk) const {
    // idea we give a template function as lambda with fixed parameters depending on
    // the region value, e.g (1*x +2) or (x + 0.1x*x), the normalization scaling c is then calcuated that
    // the integral gets 1
    double integration_sum = 0.0f;
    for(const auto&region : prior_knowledge_function_->GetKnowledgeRegion()->Partition(num_partitions_integration_)) {
        auto prior_knowledge_value = prior_knowledge_function_->GetIntegralKnowledeValue()
        auto template_scenario_integral_value = template_function_scenario_risk(region);
        integration_sum += prior_knowledge_value*template_scenario_integral_value;
    }

    double normalization_factor = 1/integration_sum;
    return ;
}

