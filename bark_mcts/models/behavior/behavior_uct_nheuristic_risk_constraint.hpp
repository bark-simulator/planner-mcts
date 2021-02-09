// Copyright (c) 2020 Julian Bernhard
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef MODULES_MODELS_BEHAVIOR_BEHAVIOR_UCT_NHEURISTIC_RISK_CONSTRAINT_HPP_
#define MODULES_MODELS_BEHAVIOR_BEHAVIOR_UCT_NHEURISTIC_RISK_CONSTRAINT_HPP_

#include <memory>
#include "bark/commons/util/util.hpp"
#include "bark_mcts/models/behavior/behavior_uct_risk_constraint.hpp"
#include "bark_mcts/models/behavior/mcts_heuristic/mcts_neural_heuristic.hpp"

namespace bark {
namespace world {
class ObservedWorld;
}
namespace models {
namespace behavior {


class BehaviorUCTNHeuristicRiskConstraint : public BehaviorUCTRiskConstraint {
 public:
  explicit BehaviorUCTNHeuristicRiskConstraint(const commons::ParamsPtr& params,
                                const std::vector<BehaviorModelPtr>& behavior_hypothesis,
                                const risk_calculation::ScenarioRiskFunctionPtr& scenario_risk_function,
                                const UctHypothesisDebugInfos& hypothesis_debug_infos,
                                const UctRiskConstraintDebugInfos& risk_constraint_debug_infos,
                                const UctBaseDebugInfos& base_debug_infos,
                                const std::string& model_file_name,
                                const bark_ml::observers::ObserverPtr& observer);

  explicit BehaviorUCTNHeuristicRiskConstraint(const commons::ParamsPtr& params,
                                const std::vector<BehaviorModelPtr>& behavior_hypothesis,
                                const risk_calculation::ScenarioRiskFunctionPtr& scenario_risk_function,
                                const std::string& model_file_name,
                                const bark_ml::observers::ObserverPtr& observer);

  virtual ~BehaviorUCTNHeuristicRiskConstraint() {}

  bark_ml::observers::ObserverPtr GetObserver() const { return observer_; }
  std::string GetModelFileName() const { return model_filename_; }

  virtual Trajectory Plan(float delta_time,
                          const world::ObservedWorld& observed_world);
  virtual std::shared_ptr<BehaviorModel> Clone() const;

 private:
  virtual void InitializeHeuristic(void* mcts) const override;

  std::string model_filename_;
  bark_ml::observers::ObserverPtr observer_;
};

inline std::shared_ptr<BehaviorModel> BehaviorUCTNHeuristicRiskConstraint::Clone() const {
  std::shared_ptr<BehaviorUCTNHeuristicRiskConstraint> model_ptr =
      std::make_shared<BehaviorUCTNHeuristicRiskConstraint>(*this);
  return model_ptr;
}

using NHeuristicMcts = mcts::Mcts<MctsStateHypothesis<MctsStateRiskConstraint>, mcts::CostConstrainedStatistic,
              mcts::HypothesisStatistic, MctsNeuralHeuristic>;

void BehaviorUCTNHeuristicRiskConstraint::InitializeHeuristic(void* mcts) const {
      LOG(INFO) << "Initializing model....";
      static_cast<NHeuristicMcts*>(mcts)->get_heuristic_function().Initialize(observer_, model_filename_);          
}


}  // namespace behavior
}  // namespace models
}  // namespace bark

#endif  // MODULES_MODELS_BEHAVIOR_BEHAVIOR_UCT_NHEURISTIC_RISK_CONSTRAINT_HPP_