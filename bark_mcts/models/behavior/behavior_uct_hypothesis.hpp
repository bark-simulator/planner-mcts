// Copyright (c) 2020 Julian Bernhard
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef MODULES_MODELS_BEHAVIOR_BEHAVIOR_UCT_HYPOTHESIS_HPP_
#define MODULES_MODELS_BEHAVIOR_BEHAVIOR_UCT_HYPOTHESIS_HPP_

#include <memory>
#include "bark_mcts/models/behavior/behavior_uct_hypothesis_base.hpp"
#include "bark_mcts/models/behavior/mcts_state/mcts_state_hypothesis.hpp"

namespace bark {
namespace world {
class ObservedWorld;
}
namespace models {
namespace behavior {

class BehaviorUCTHypothesis : public BehaviorUCTHypothesisBase<MctsStateHypothesis<>> {
 public:
  explicit BehaviorUCTHypothesis(const commons::ParamsPtr& params,
                                const std::vector<BehaviorModelPtr>& behavior_hypothesis);

  virtual ~BehaviorUCTHypothesis() {}

    virtual Trajectory Plan(float delta_time,
                          const world::ObservedWorld& observed_world);

  virtual std::shared_ptr<BehaviorModel> Clone() const;
};

inline std::shared_ptr<BehaviorModel> BehaviorUCTHypothesis::Clone() const {
  std::shared_ptr<BehaviorUCTHypothesis> model_ptr =
      std::make_shared<BehaviorUCTHypothesis>(*this);
  return model_ptr;
}


}  // namespace behavior
}  // namespace models
}  // namespace bark

#endif  // MODULES_MODELS_BEHAVIOR_BEHAVIOR_UCT_HYPOTHESIS_HPP_