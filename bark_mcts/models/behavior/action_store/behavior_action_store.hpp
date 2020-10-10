// Copyright (c) 2020 Julian Bernhard
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef MODULES_MODELS_BEHAVIOR_ACTION_STORE_BEHAVIOR_ACTION_STORE_HPP_
#define MODULES_MODELS_BEHAVIOR_ACTION_STORE_BEHAVIOR_ACTION_STORE_HPP_

#include <boost/functional/hash.hpp>
#include <unordered_map>

#include "bark/models/behavior/behavior_model.hpp"

namespace bark {
namespace world {
class ObservedWorld;
}  
namespace models {
namespace behavior {


using dynamic::Trajectory;
using dynamic::Input;


class BehaviorActionStore : public BehaviorModel {
 public:
  BehaviorActionStore(const commons::ParamsPtr& params)
      : BehaviorModel(params),
      trajectory_store_(),
      active_behavior_() {}

  virtual ~BehaviorActionStore() {}

  ActionHash Store(const Action& action, const Trajectory& trajectory, const BehaviorStatus& status);
  std::tuple<Trajectory, Action, BehaviorStatus> Retrieve(const ActionHash& action_hash) const;

  void MakeBehaviorActive(const ActionHash& action_hash) {
    active_behavior_ = action_hash;
  }

  std::string print_stored_hashes() const;

  virtual std::shared_ptr<BehaviorModel> Clone() const; 

  virtual Trajectory Plan(float delta_time, const bark::world::ObservedWorld& observed_world);

  private:
    std::unordered_map<ActionHash, std::tuple<Trajectory, Action, BehaviorStatus>> trajectory_store_;
    ActionHash active_behavior_;
};

inline std::shared_ptr<BehaviorModel> BehaviorActionStore::Clone() const {
  std::shared_ptr<BehaviorActionStore> model_ptr =
      std::make_shared<BehaviorActionStore>(*this);
  return model_ptr;
}


typedef std::shared_ptr<BehaviorActionStore> BehaviorActionStorePtr;

struct ActionHasher : public boost::static_visitor<std::size_t>
{
    template<typename T>
    std::size_t operator()(const T& action) const { return boost::hash<T>()(action); }
    std::size_t operator()(const LonLatAction& action) const {
          std::size_t hash = boost::hash<double>()(action.acc_lat);
          boost::hash_combine(hash, action.acc_lon); 
          return hash;
    }
    std::size_t operator()(const Input& x) const { return std::size_t(); }
};

inline ActionHash ActionToHash(const Action& action) {
  return boost::apply_visitor(ActionHasher(), action);
}


}  // namespace behavior
}  // namespace models
}  // namespace bark

#endif  // MODULES_MODELS_BEHAVIOR_ACTION_STORE_BEHAVIOR_ACTION_STORE_HPP_
