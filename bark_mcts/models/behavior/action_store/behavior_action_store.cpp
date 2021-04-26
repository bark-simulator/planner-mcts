// Copyright (c) 2020 Julian Bernhard
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "bark_mcts/models/behavior/action_store/behavior_action_store.hpp"


namespace bark {
namespace models {
namespace behavior {

using dynamic::Trajectory;


ActionHash BehaviorActionStore::Store(const Action& action, const Trajectory& trajectory,
                                      const BehaviorStatus& status) {
  const auto action_hash = ActionToHash(action);
  auto it = trajectory_store_.find(action_hash);
  if(it == trajectory_store_.end()) {
    trajectory_store_.emplace(std::make_pair(action_hash, 
              std::make_tuple(trajectory, action, status)));
  }
  return action_hash;
}

std::tuple<Trajectory, Action, BehaviorStatus> BehaviorActionStore::Retrieve(const ActionHash& action_hash) const {
  auto it = trajectory_store_.find(action_hash);
  if(it == trajectory_store_.end()) {
    LOG(FATAL) << "Action " << action_hash << "not found in store. Available: " << print_stored_hashes();
  }
  return it->second;
}

std::string BehaviorActionStore::print_stored_hashes() const {
  std::stringstream ss;
  ss << "[";
  for (const auto& stored : trajectory_store_) {
    ss << stored.first << ", ";
  }
  ss << "]";
  return ss.str();
}

Trajectory BehaviorActionStore::Plan(double min_planning_time, const bark::world::ObservedWorld& observed_world) {
  const auto& tuple = Retrieve(active_behavior_);
  SetLastTrajectory(std::get<0>(tuple));
  SetLastAction(std::get<1>(tuple));
  SetBehaviorStatus(std::get<2>(tuple));
  return std::get<0>(tuple);
}



}  // namespace behavior
}  // namespace models
}  // namespace bark
