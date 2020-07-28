// Copyright (c) 2020 Julian Bernhard
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.


#ifndef MODULES_MODELS_BEHAVIOR_RISK_CALCULATION_PRIOR_KNOWLEDGE_REGION_HPP_
#define MODULES_MODELS_BEHAVIOR_RISK_CALCULATION_PRIOR_KNOWLEDGE_REGION_HPP_

#include <unordered_map>
#include <functional>
#include <vector>

namespace bark {
namespace models {
namespace behavior {
namespace risk_calculation {


typedef float KnowledgeValue;
typedef float RegionValueType;
typedef std::string DimensionName;
typedef std::unordered_map<DimensionName, RegionValueType> RegionValue;
typedef std::unordered_map<DimensionName, std::pair<RegionValueType, RegionValueType>> RegionBoundaries;

// to do -> do we need this actually
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

typedef std::function<KnowledgeValue(RegionBoundaries)> KnowledgeFunction;


} // namespace risk calculation
} // namespace behavior
} // namespace models
} // namespace bark

#endif // MODULES_MODELS_BEHAVIOR_RISK_CALCULATION_PRIOR_KNOWLEDGE_REGION_HPP_