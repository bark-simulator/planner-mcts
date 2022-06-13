// Copyright (c) 2020 Julian Bernhard
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef MODULES_MODELS_BEHAVIOR_BEHAVIOR_UCT_COOPERATIVE_HPP_
#define MODULES_MODELS_BEHAVIOR_BEHAVIOR_UCT_COOPERATIVE_HPP_

#include <memory>
#include "bark_mcts/models/behavior/behavior_uct_base.hpp"

namespace bark {
namespace world {
class ObservedWorld;
}
namespace models {
namespace behavior {

class BehaviorUCTCooperative : public BehaviorUCTBase {
 public:
  explicit BehaviorUCTCooperative(const commons::ParamsPtr& params);

  virtual ~BehaviorUCTCooperative() {}

    virtual Trajectory Plan(double min_planning_time,
                          const world::ObservedWorld& observed_world);

  virtual std::shared_ptr<BehaviorModel> Clone() const;
};

inline std::shared_ptr<BehaviorModel> BehaviorUCTCooperative::Clone() const {
  std::shared_ptr<BehaviorUCTCooperative> model_ptr =
      std::make_shared<BehaviorUCTCooperative>(*this);
  return model_ptr;
}


}  // namespace behavior
}  // namespace models
}  // namespace bark

#endif  // MODULES_MODELS_BEHAVIOR_BEHAVIOR_UCT_COOPERATIVE_HPP_