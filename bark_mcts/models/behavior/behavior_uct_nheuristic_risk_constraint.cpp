#include "bark_mcts/models/behavior/behavior_uct_nheuristic_risk_constraint.hpp"
#include "mcts/heuristics/random_heuristic.h"
#include "mcts/mcts.h"
#include "mcts/statistics/uct_statistic.h"
#include "mcts/hypothesis/hypothesis_statistic.h"

#include "bark/world/observed_world.hpp"

namespace bark {
namespace models {
namespace behavior {

BehaviorUCTNHeuristicRiskConstraint::BehaviorUCTNHeuristicRiskConstraint(const commons::ParamsPtr& params,
                                const std::vector<BehaviorModelPtr>& behavior_hypothesis,
                                const risk_calculation::ScenarioRiskFunctionPtr& scenario_risk_function,
                                const UctHypothesisDebugInfos& hypothesis_debug_infos,
                                const UctRiskConstraintDebugInfos& risk_constraint_debug_infos,
                                const UctBaseDebugInfos& base_debug_infos) :
                                BehaviorUCTRiskConstraint(params,
                                                          behavior_hypothesis,
                                                          scenario_risk_function,
                                                          hypothesis_debug_infos,
                                                          risk_constraint_debug_infos,
                                                          base_debug_infos),
                                model_filename_(GetParams()->AddChild("BehaviorUctNHeuristicRiskConstraint")
                                    ->GetString("ModelFileName", "Name of neural network pytorch scriptfile", ""))  {}

BehaviorUCTNHeuristicRiskConstraint::BehaviorUCTNHeuristicRiskConstraint(const commons::ParamsPtr& params,
                                const std::vector<BehaviorModelPtr>& behavior_hypothesis,
                                const risk_calculation::ScenarioRiskFunctionPtr& scenario_risk_function) :
                                BehaviorUCTRiskConstraint(params, behavior_hypothesis, scenario_risk_function,
                                    UctHypothesisDebugInfos(), UctRiskConstraintDebugInfos(), UctBaseDebugInfos()) {}

dynamic::Trajectory BehaviorUCTNHeuristicRiskConstraint::Plan(
    float delta_time, const world::ObservedWorld& observed_world) {
      return PlanWithMcts<MctsStateHypothesis<MctsStateRiskConstraint>, mcts::CostConstrainedStatistic,
              mcts::HypothesisStatistic, MctsNeuralHeuristic>(delta_time, observed_world);
}

using NHeuristicMcts = mcts::Mcts<MctsStateHypothesis<MctsStateRiskConstraint>, mcts::CostConstrainedStatistic,
              mcts::HypothesisStatistic, MctsNeuralHeuristic>;
template <>
void BehaviorUCTNHeuristicRiskConstraint::InitializeHeuristic<NHeuristicMcts>
              (NHeuristicMcts& mcts) const {
                

}


}  // namespace behavior
}  // namespace models
}  // namespace bark