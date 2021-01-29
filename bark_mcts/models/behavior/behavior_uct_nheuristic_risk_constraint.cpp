#define MCTS_EXPECT_TRUE(cond) BARK_EXPECT_TRUE(cond)
#include "bark_mcts/models/behavior/behavior_uct_risk_constraint.hpp"
#include "mcts/heuristics/random_heuristic.h"
#include "mcts/mcts.h"
#include "mcts/statistics/uct_statistic.h"
#include "mcts/hypothesis/hypothesis_statistic.h"

#include "bark/world/observed_world.hpp"

namespace bark {
namespace models {
namespace behavior {

BehaviorUCTRiskConstraint::BehaviorUCTRiskConstraint(const commons::ParamsPtr& params,
                                const std::vector<BehaviorModelPtr>& behavior_hypothesis,
                                const risk_calculation::ScenarioRiskFunctionPtr& scenario_risk_function,
                                const UctHypothesisDebugInfos& hypothesis_debug_infos,
                                const UctRiskConstraintDebugInfos& risk_constraint_debug_infos,
                                const UctBaseDebugInfos& base_debug_infos) :
                                BehaviorUCTHypothesisBase(
                                        params, behavior_hypothesis, hypothesis_debug_infos, base_debug_infos),
                                UctRiskConstraintDebugInfos(risk_constraint_debug_infos),
                                default_available_risk_([&]() {
                                const auto float_vec = GetParams()->AddChild("BehaviorUctRiskConstraint")
                                    ->GetListFloat("DefaultAvailableRisk", "Risk used when belief not initialized", {0.0f});
                                std::vector<double> double_vec(float_vec.size());
                                std::transform(float_vec.begin(), float_vec.end(), double_vec.begin(),
                                       [](const float& v){return static_cast<double>(v);});
                                return double_vec;
                                }()),
                                current_scenario_risk_(default_available_risk_),
                                estimate_scenario_risk_(GetParams()->AddChild("BehaviorUctRiskConstraint")
                                      ->GetBool("EstimateScenarioRisk", "Should scenario risk be estimated from scenario risk function", false)),
                                update_scenario_risk_(GetParams()->AddChild("BehaviorUctRiskConstraint")
                                      ->GetBool("UpdateScenarioRisk", "Should scenario risk be estimated from scenario risk function", true)),
                                initialized_available_risk_(!estimate_scenario_risk_),
                                scenario_risk_function_(scenario_risk_function) {}

BehaviorUCTRiskConstraint::BehaviorUCTRiskConstraint(const commons::ParamsPtr& params,
                                const std::vector<BehaviorModelPtr>& behavior_hypothesis,
                                const risk_calculation::ScenarioRiskFunctionPtr& scenario_risk_function) :
                                BehaviorUCTRiskConstraint(params, behavior_hypothesis, scenario_risk_function,
                                    UctHypothesisDebugInfos(), UctRiskConstraintDebugInfos(), UctBaseDebugInfos()) {}

dynamic::Trajectory BehaviorUCTRiskConstraint::Plan(
    float delta_time, const world::ObservedWorld& observed_world) {
      return PlanWithMcts<MctsStateHypothesis<MctsStateRiskConstraint>, mcts::CostConstrainedStatistic,
              mcts::HypothesisStatistic, mcts::RandomHeuristic>(delta_time, observed_world);
}

 mcts::Cost BehaviorUCTRiskConstraint::CalculateAvailableScenarioRisk() const {
  // First calculate mean of belief for each hypothesis
  const std::unordered_map<mcts::AgentIdx, std::vector<mcts::Belief>>& beliefs
                        = belief_tracker_.get_beliefs();
  std::vector<mcts::Belief> belief_sum(beliefs.begin()->second.size(), 0.0f);
  for (const auto& agent_beliefs : beliefs) {
    for (mcts::HypothesisId hyp_id = 0; hyp_id < agent_beliefs.second.size(); ++hyp_id) {
      belief_sum[hyp_id] += agent_beliefs.second.at(hyp_id);
    }
  }
  VLOG(3) << "Calculating available scenario risk ...";
  mcts::Cost available_risk = 0.0f;
  for (mcts::HypothesisId hyp_id = 0; hyp_id < behavior_hypotheses_.size(); ++hyp_id) {
    const auto hypothesis = std::dynamic_pointer_cast<BehaviorHypothesis>(behavior_hypotheses_.at(hyp_id));
    const auto& integrated_risk = scenario_risk_function_->CalculateIntegralValue(*hypothesis);
    const auto& behavior_space_area = risk_calculation::CalculateRegionBoundariesArea(hypothesis->GetDefinition());
    available_risk += 1.0 / belief_sum.size() * belief_sum.at(hyp_id) * integrated_risk / behavior_space_area;
    VLOG(3) << "integrated risk: " << integrated_risk << ", behavior space area: " << behavior_space_area << ", belief sum: " <<  belief_sum.at(hyp_id);
  }
  VLOG(3) << "available risk: " << available_risk;
  return available_risk;
 }

}  // namespace behavior
}  // namespace models
}  // namespace bark