// Copyright (c) 2020 Julian Bernhard
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.


#ifndef MODULES_MODELS_BEHAVIOR_RISK_CALCULATION_PRIOR_KNOWLEDGE_REGION_HPP_
#define MODULES_MODELS_BEHAVIOR_RISK_CALCULATION_PRIOR_KNOWLEDGE_REGION_HPP_

namespace bark {
namespace models {
namespace behavior {
namespace prior_knowledge {


typedef double KnowledgeValue;
typedef double RegionValueType;
typedef double DimensionName;
typedef std::unordered_map<DimensionName, RegionValueType> RegionValue;
typedef std::unordered_map<DimensionName, std::pair<RegionValueType, RegionValueType>> RegionBoundaries;

// to do -> do we need this actually
class KnowledgeRegion {
   KnowledgeRegion(const RegionBoundaries& region_definition)
       : region_definition_(region_definition) {}
   RegionBoundaries GetDefinition() const;

   std::vector<KnowledgeRegion> Partition(unsigned int num_partitions);

   private:
    RegionBoundaries region_definition_;
}


} // namespace risk calculation
} // namespace behavior
} // namespace models
} // namespace bark

#endif // MODULES_MODELS_BEHAVIOR_RISK_CALCULATION_PRIOR_KNOWLEDGE_REGION_HPP_