// Copyright (c) 2020 Julian Bernhard
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.


#ifndef MODULES_MODELS_BEHAVIOR_RISK_CALCULATION_SCENARIO_RISK_FUNCTION_HPP_
#define MODULES_MODELS_BEHAVIOR_RISK_CALCULATION_SCENARIO_RISK_FUNCTION_HPP_

#include "bark_mcts/models/behavior/risk_calculation/prior_knowledge_function.hpp"

namespace bark {
namespace models {
namespace behavior {
namespace risk_calculation {

class ScenarioRiskFunction {
    ScenarioRiskFunction(const KnowledgeFunction& risk_function_unnormalized,
                        const double& normalization_constant_) : 
                indefinite_integral_(indefinite_integral),
                normalization_constant_(normalization_constant) {}
    Probability CalculateMeanAvailableScenarioRisk(const KnowledgeRegion&) const {+
            return normalization_constant_ * GetValueTemplateFunction(region);
    }
    // Defines the template function
    KnowledgeValue GetIntegralValueTemplateFunction(const KnowledgeRegion& region) const {
        return region.
    }
    KnowledgeValue GetValueFunction(const KnowledgeRegion& region) const {

    }

    private:
    // Holds a lambda passed from python of the indefinite integral function
    // e.g. if the scenario risk template is 0.1*x^2 then the lambda must be 0.1/3*x^3 
    const std::function<KnowledgeValue(KnowledgeRegion)> indefinite_integral_;
    double normalization_constant_;
}


} // namespace risk calculation
} // namespace behavior
} // namespace models
} // namespace bark

#endif // MODULES_MODELS_BEHAVIOR_RISK_CALCULATION_SCENARIO_RISK_FUNCTION_HPP_