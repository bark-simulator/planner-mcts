// Copyright (c) 2020 Julian Bernhard
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef MODULES_MODELS_BEHAVIOR_BEHAVIOR_UCT_RISK_CONSTRAINT_HPP_
#define MODULES_MODELS_BEHAVIOR_BEHAVIOR_UCT_RISK_CONSTRAINT_HPP_

#include <memory>
#include "bark_mcts/models/behavior/behavior_uct_hypothesis_base.hpp"
#include "bark_mcts/models/behavior/mcts_state/mcts_state_risk_constraint.hpp"
#include "bark_mcts/models/behavior/risk_calculation/scenario_risk_function.hpp"
#include "mcts/hypothesis/hypothesis_belief_tracker.h"

namespace bark {
namespace world {
class ObservedWorld;
}
namespace models {
namespace behavior {

class BehaviorUCTRiskConstraint : public BehaviorUCTHypothesisBase<MctsStateRiskConstraint> {
 public:
  explicit BehaviorUCTRiskConstraint(const commons::ParamsPtr& params,
                                const std::vector<BehaviorModelPtr>& behavior_hypothesis,
                                const risk_calculation::ScenarioRiskFunctionPtr& scenario_risk_function);

  virtual ~BehaviorUCTRiskConstraint() {}

  virtual Trajectory Plan(float delta_time,
                          const world::ObservedWorld& observed_world);

  virtual std::shared_ptr<BehaviorModel> Clone() const;

  risk_calculation::ScenarioRiskFunctionPtr GetScenarioRiskFunction() const { return scenario_risk_function_; }

  protected:
    mcts::Cost CalculateAvailableScenarioRisk() const;

    mcts::Cost default_available_risk_;
    mcts::Cost current_scenario_risk_;
    bool estimate_scenario_risk_; // Should scenario risk be estimated from scenario risk function
    bool initialized_available_risk_; // Was scenario risk initialized from scenario risk function after belief was initialized;
    risk_calculation::ScenarioRiskFunctionPtr scenario_risk_function_;
};

inline std::shared_ptr<BehaviorModel> BehaviorUCTRiskConstraint::Clone() const {
  std::shared_ptr<BehaviorUCTRiskConstraint> model_ptr =
      std::make_shared<BehaviorUCTRiskConstraint>(*this);
  return model_ptr;
}

}  // namespace behavior
}  // namespace models
}  // namespace bark

#endif  // MODULES_MODELS_BEHAVIOR_BEHAVIOR_UCT_RISK_CONSTRAINT_HPP_