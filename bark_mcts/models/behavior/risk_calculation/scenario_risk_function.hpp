// Copyright (c) 2020 Julian Bernhard
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.


#ifndef MODULES_MODELS_BEHAVIOR_RISK_CALCULATION_SCENARIO_RISK_FUNCTION_HPP_
#define MODULES_MODELS_BEHAVIOR_RISK_CALCULATION_SCENARIO_RISK_FUNCTION_HPP_

#include "bark/commons/distribution/distribution.hpp"
#include "bark_mcts/models/behavior/risk_calculation/knowledge_function_template.hpp"
#include "bark_mcts/models/behavior/risk_calculation/prior_knowledge_region.hpp"


namespace bark {
namespace models {
namespace behavior {
namespace risk_calculation {

typedef std::function<KnowledgeValue(RegionBoundaries)> ScenarioRiskFunctionTemplate;

class ScenarioRiskFunction {
  public:
    ScenarioRiskFunction(const ScenarioRiskFunctionTemplate& risk_function_unnormalized,
                        const double& normalization_constant) : 
                risk_function_unnormalized_(risk_function_unnormalized),
                normalization_constant_(normalization_constant) {}

    bark::commons::Probability CalculateMeanAvailableScenarioRisk(const KnowledgeRegion& region) const {
            return normalization_constant_ * GetIntegralValueTemplateFunction(region);
    }

    // Defines the template function
    KnowledgeValue GetIntegralValueTemplateFunction(const KnowledgeRegion& region) const {
    }

    double GetNormalizationConstant() const { return normalization_constant_;}

    ScenarioRiskFunctionTemplate GetScenarioRiskFunctionTemplate() const { return risk_function_unnormalized_;}

  private:
    // Holds a lambda passed from python of the indefinite integral function
    // e.g. if the scenario risk template is 0.1*x^2 then the lambda must be 0.1/3*x^3 
    const ScenarioRiskFunctionTemplate risk_function_unnormalized_;
    double normalization_constant_;
};

typedef std::shared_ptr<ScenarioRiskFunction> ScenarioRiskFunctionPtr;

} // namespace risk calculation
} // namespace behavior
} // namespace models
} // namespace bark

#endif // MODULES_MODELS_BEHAVIOR_RISK_CALCULATION_SCENARIO_RISK_FUNCTION_HPP_