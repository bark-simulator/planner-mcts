// Copyright (c) 2020 Julian Bernhard
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "bark_mcts/models/behavior/risk_calculation/prior_knowledge_region.hpp"

using namespace bark::models::behavior::risk_calculation;

std::vector<PriorKnowledgeRegion> PriorKnowledgeRegion::Partition(unsigned int num_partitions_per_dimension) const {
  std::unordered_map<DimensionName, std::vector<std::pair<RegionValueType, RegionValueType>>> dimension_partitions;
  for (const auto& dimension : region_definition_) {
    const auto& dim_name = dimension.first;
    const auto& dim_range = dimension.second.second - dimension.second.first;
    const auto& part_width = dim_range/num_partitions_per_dimension;
    for(unsigned int i = 0; i < num_partitions_per_dimension; ++i) {
        dimension_partitions[dim_name].push_back(std::make_pair(
                     dimension.second.first + part_width*i,
                     dimension.second.first + part_width*(i+1)));
    }
  }

  std::vector<std::unordered_map<DimensionName, std::pair<RegionValueType, RegionValueType>>> region_boundary_collected;
  for (const auto& dim_part : dimension_partitions) {
    const auto& dim_name = dim_part.first;
    const auto& partitions = dim_part.second;
    if (region_boundary_collected.empty()) {
      // First dimensions, now combinations possible
      for (const auto& part : partitions) {
        RegionBoundaries boundary;
        boundary[dim_name] = part;
        region_boundary_collected.push_back(boundary);
      }
    } else {
      // Combine this vector with all previous combinations
      std::vector<std::unordered_map<DimensionName, std::pair<RegionValueType, RegionValueType>>> tmp_collected;
      for (const auto& collected : region_boundary_collected) {
        for (const auto& part : partitions) {
          auto new_collected = collected;
          new_collected[dim_name] = part;
          tmp_collected.push_back(new_collected);
        }
      }
      region_boundary_collected = tmp_collected;
    }
  }

  std::vector<PriorKnowledgeRegion> partitions;
      for (const auto& region_boundary : region_boundary_collected) {
        partitions.push_back(PriorKnowledgeRegion(region_boundary));
      }
      return partitions;
} 


double PriorKnowledgeRegion::CalculateArea() const {
  return CalculateRegionBoundariesArea(region_definition_);
}