// Copyright (c) 2020 Julian Bernhard
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.


#ifndef MODULES_MODELS_BEHAVIOR_RISK_CALCULATION_LINEAR_KNOWLEDGE_FUNCTION_DEFINITION_HPP_
#define MODULES_MODELS_BEHAVIOR_RISK_CALCULATION_LINEAR_KNOWLEDGE_FUNCTION_DEFINITION_HPP_

#include "bark_mcts/models/behavior/risk_calculation/prior_knowledge_region.hpp"
#include "bark/commons/distribution/distribution.hpp"

namespace bark {
namespace models {
namespace behavior {
namespace risk_calculation {


class LinearKnowledgeFunctionDefinition : public KnowledgeFunctionDefinition {
  public:
     LinearKnowledgeFunctionDefinition(const PriorKnowledgeRegion& supporting_region,
                                const ParamsPtr& params) :
                            KnowledgeFunctionDefinition(supporting_region, params),
                            function_params_() {
      for(const auto& region_def : supporting_region.GetDefinition()) {
        function_params_[region_def.first] = LinearFunctionParams{
            params->GetReal("LinearKnowledgeFunction::" + region_def.first + "::a", "parameter a", 1.0),
            params->GetReal("LinearKnowledgeFunction::" + region_def.first + "::b", "parameter b", 1.0)};
      }
    }

    virtual KnowledgeValue CalculateIntegral(const RegionBoundaries& integral_region) const override {
      KnowledgeValue mean_in_region = 0.0;
      for (const auto& region : integral_region) {
          const auto& range = region.second;
          const auto& rp = function_params_.at(region.first);
          mean_in_region += ( (rp.a*range.first + rp.b) + (rp.a*range.second + rp.b) ) / 2.0;
      }
      return mean_in_region / integral_region.size() * CalculateRegionBoundariesArea(integral_region);
    }

    virtual KnowledgeSample Sample(const PriorKnowledgeRegion& sampling_region) const override {
      throw std::runtime_error("Sampling not implemented for linear function definition");
    };

    KnowledgeValue Linear1dFunctionMean(const double& a, const double& b, const std::pair<RegionValueType, RegionValueType>& range) const {
      return ( (a*range.first + b) + (a*range.second + b) ) / 2.0; // consider the mean between the two region points 
    }

    private:
      typedef struct {
        double a;
        double b;
      } LinearFunctionParams;
      std::unordered_map<std::string, LinearFunctionParams> function_params_;
};

typedef std::shared_ptr<KnowledgeFunctionDefinition> KnowledgeFunctionDefinitionPtr;

} // namespace risk calculation
} // namespace behavior
} // namespace models
} // namespace bark

#endif // MODULES_MODELS_BEHAVIOR_RISK_CALCULATION_LINEAR_KNOWLEDGE_FUNCTION_DEFINITION_HPP_