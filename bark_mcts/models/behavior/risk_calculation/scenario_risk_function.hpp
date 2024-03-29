// Copyright (c) 2020 Julian Bernhard
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.


#ifndef MODULES_MODELS_BEHAVIOR_RISK_CALCULATION_SCENARIO_RISK_FUNCTION_HPP_
#define MODULES_MODELS_BEHAVIOR_RISK_CALCULATION_SCENARIO_RISK_FUNCTION_HPP_

#include "bark/commons/distribution/distribution.hpp"
#include "bark_mcts/models/behavior/risk_calculation/knowledge_function_definition/knowledge_function_definition.hpp"
#include "bark_mcts/models/behavior/risk_calculation/prior_knowledge_region.hpp"


namespace bark {
namespace models {
namespace behavior {
namespace risk_calculation {

class ScenarioRiskFunction {
  public:
    ScenarioRiskFunction(const KnowledgeFunctionDefinitionPtr& risk_function_definition,
                        const double& normalization_constant) : 
                risk_function_definition_(risk_function_definition),
                normalization_constant_(normalization_constant) {}

    bark::commons::Probability CalculateIntegralValue(const KnowledgeRegion& region) const {
            return normalization_constant_ * GetIntegralValueTemplateFunction(region);
    }

    // Defines the template function
    KnowledgeValue GetIntegralValueTemplateFunction(const KnowledgeRegion& region) const {
      return risk_function_definition_->CalculateIntegral(region.GetDefinition());
    }

    double GetNormalizationConstant() const { return normalization_constant_;}

    KnowledgeFunctionDefinitionPtr GetRiskFunctionDefinition() const { return risk_function_definition_;}

  private:
    KnowledgeFunctionDefinitionPtr risk_function_definition_;
    double normalization_constant_;
};

typedef std::shared_ptr<ScenarioRiskFunction> ScenarioRiskFunctionPtr;

} // namespace risk calculation
} // namespace behavior
} // namespace models
} // namespace bark

#endif // MODULES_MODELS_BEHAVIOR_RISK_CALCULATION_SCENARIO_RISK_FUNCTION_HPP_