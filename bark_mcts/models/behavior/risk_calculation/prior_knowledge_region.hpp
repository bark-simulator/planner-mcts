// Copyright (c) 2020 Julian Bernhard
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.


#ifndef MODULES_MODELS_BEHAVIOR_RISK_CALCULATION_PRIOR_KNOWLEDGE_REGION_HPP_
#define MODULES_MODELS_BEHAVIOR_RISK_CALCULATION_PRIOR_KNOWLEDGE_REGION_HPP_

#include "bark_mcts/models/behavior/risk_calculation/common.hpp"

namespace bark {
namespace models {
namespace behavior {
namespace risk_calculation {

inline double CalculateRegionBoundariesArea(const RegionBoundaries& region_boundaries) {
  double area = 1.0;
  for (const auto& region: region_boundaries) {
    if (region.second.second > region.second.first) {
      area *= (region.second.second - region.second.first);
    }
  }
  return area;
}

class KnowledgeRegion {
  public:
    KnowledgeRegion() {}
    virtual RegionBoundaries GetDefinition() const = 0;
};

class PriorKnowledgeRegion : public KnowledgeRegion {
  public:
    PriorKnowledgeRegion(const RegionBoundaries& region_definition)
        : region_definition_(region_definition) {}
    virtual RegionBoundaries GetDefinition() const { return region_definition_;}

    double CalculateArea() const;

    std::vector<PriorKnowledgeRegion> Partition(unsigned int num_partitions_per_dimension) const;

   private:
      RegionBoundaries region_definition_;
};


} // namespace risk calculation
} // namespace behavior
} // namespace models
} // namespace bark

#endif // MODULES_MODELS_BEHAVIOR_RISK_CALCULATION_PRIOR_KNOWLEDGE_REGION_HPP_