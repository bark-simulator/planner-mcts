#include "bark_mcts/models/behavior/behavior_uct_nheuristic_risk_constraint.hpp"
#include "mcts/heuristics/random_heuristic.h"
#include "mcts/mcts.h"
#include "mcts/statistics/uct_statistic.h"
#include "mcts/hypothesis/hypothesis_statistic.h"
#include "bark_mcts/models/behavior/mcts_statistics/mcts_neural_cost_constrained_statistic.hpp"

#include "bark/world/observed_world.hpp"

namespace mcts {
  std::shared_ptr<bark_ml::lib_fqf_iqn_qrdqn::ModelLoader> NeuralCostConstrainedStatistic::model_loader_;
  std::shared_ptr<bark_ml::observers::BaseObserver> NeuralCostConstrainedStatistic::observer_;
  bark_ml::lib_fqf_iqn_qrdqn::NNToValueConverterPtr NeuralCostConstrainedStatistic::nn_to_value_converter_;
}

namespace bark {
namespace models {
namespace behavior {

BehaviorUCTNHeuristicRiskConstraint::BehaviorUCTNHeuristicRiskConstraint(const commons::ParamsPtr& params,
                                const std::vector<BehaviorModelPtr>& behavior_hypothesis,
                                const risk_calculation::ScenarioRiskFunctionPtr& scenario_risk_function,
                                const UctHypothesisDebugInfos& hypothesis_debug_infos,
                                const UctRiskConstraintDebugInfos& risk_constraint_debug_infos,
                                const UctBaseDebugInfos& base_debug_infos,
                                const std::string& model_file_name,
                                const bark_ml::observers::ObserverPtr& observer,
                                const bark_ml::lib_fqf_iqn_qrdqn::NNToValueConverterPtr& nn_to_value_converter) :
                                BehaviorUCTRiskConstraint(params,
                                                          behavior_hypothesis,
                                                          scenario_risk_function,
                                                          hypothesis_debug_infos,
                                                          risk_constraint_debug_infos,
                                                          base_debug_infos),
                                model_filename_(model_file_name),
                                observer_(observer),
                                nn_to_value_converter_(nn_to_value_converter)  {}

BehaviorUCTNHeuristicRiskConstraint::BehaviorUCTNHeuristicRiskConstraint(const commons::ParamsPtr& params,
                                const std::vector<BehaviorModelPtr>& behavior_hypothesis,
                                const risk_calculation::ScenarioRiskFunctionPtr& scenario_risk_function,
                                const std::string& model_file_name,
                                const bark_ml::observers::ObserverPtr& observer,
                                const bark_ml::lib_fqf_iqn_qrdqn::NNToValueConverterPtr& nn_to_value_converter) :
                                BehaviorUCTNHeuristicRiskConstraint(params, behavior_hypothesis, scenario_risk_function,
                                    UctHypothesisDebugInfos(), UctRiskConstraintDebugInfos(), UctBaseDebugInfos(),
                                     model_file_name, observer, nn_to_value_converter) {}

dynamic::Trajectory BehaviorUCTNHeuristicRiskConstraint::Plan(
    double min_planning_time, const world::ObservedWorld& observed_world) {
      return PlanWithMcts<MctsStateHypothesis<MctsStateRiskConstraint>, mcts::NeuralCostConstrainedStatistic,
              mcts::HypothesisStatistic, mcts::RandomHeuristic>(min_planning_time, observed_world);
}


}  // namespace behavior
}  // namespace models
}  // namespace bark