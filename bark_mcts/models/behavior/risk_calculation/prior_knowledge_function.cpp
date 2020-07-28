// Copyright (c) 2020 Julian Bernhard
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "bark_mcts/models/behavior/risk_calculation/prior_knowledge_function.hpp"

using namespace bark::models::behavior::risk_calculation;

KnowledgeValue PriorKnowledgeFunction::GetIntegralKnowledeValue(const RegionBoundaries& knowledge_region) const {
  return knowledge_function_(knowledge_region);
}

ScenarioRiskFunctionPtr PriorKnowledgeFunction::CalculateScenarioRiskFunction(const KnowledgeFunction& template_scenario_risk_function) const {
    // idea we give a template function as lambda with fixed parameters depending on
    // the region value, e.g (1*x +2) or (x + 0.1x*x), the normalization scaling c is then calcuated that
    // the integral gets 1
    double integration_sum = 0.0f;
    for(const auto&region : prior_knowledge_region_.Partition(num_partitions_integration_)) {
        auto prior_knowledge_value = GetIntegralKnowledeValue(region.GetDefinition());
        auto template_scenario_integral_value = template_scenario_risk_function(region.GetDefinition());
        integration_sum += prior_knowledge_value*template_scenario_integral_value;
    }

    double normalization_factor = 1/integration_sum;
    return std::make_shared<ScenarioRiskFunction>(template_scenario_risk_function, normalization_factor);
}

