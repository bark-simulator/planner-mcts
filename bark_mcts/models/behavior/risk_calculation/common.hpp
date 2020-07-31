// Copyright (c) 2020 Julian Bernhard
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef MODULES_MODELS_BEHAVIOR_RISK_CALCULATION_COMMON_HPP_
#define MODULES_MODELS_BEHAVIOR_RISK_CALCULATION_COMMON_HPP_

#include <unordered_map>
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

} // namespace risk calculation
} // namespace behavior
} // namespace models
} // namespace bark

#endif // MODULES_MODELS_BEHAVIOR_RISK_CALCULATION_COMMON_HPP_