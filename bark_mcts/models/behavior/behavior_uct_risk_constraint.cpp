#define MCTS_EXPECT_TRUE(cond) BARK_EXPECT_TRUE(cond)
#include "mcts/cost_constrained/cost_constrained_statistic.h"
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
                                const risk_calculation::ScenarioRiskFunctionPtr& scenario_risk_function) :
                                BehaviorUCTHypothesisBase(params, behavior_hypothesis),
                                default_available_risk_(GetParams()->AddChild("BehaviorUctRiskConstraint")
                                      ->GetReal("DefaultAvailableRisk", "Risk used when belief not initialized", 0.0f)),
                                current_scenario_risk_(default_available_risk_),
                                estimate_scenario_risk_(GetParams()->AddChild("BehaviorUctRiskConstraint")
                                      ->GetBool("EstimateScenarioRisk", "Should scenario risk be estimated from scenario risk function", false)),
                                update_scenario_risk_(GetParams()->AddChild("BehaviorUctRiskConstraint")
                                      ->GetBool("UpdateScenarioRisk", "Should scenario risk be estimated from scenario risk function", true)),
                                initialized_available_risk_(!estimate_scenario_risk_),
                                scenario_risk_function_(scenario_risk_function) {}

dynamic::Trajectory BehaviorUCTRiskConstraint::Plan(
    float delta_time, const world::ObservedWorld& observed_world) {
  ObservedWorldPtr mcts_observed_world = BehaviorUCTBase::FilterAgents(observed_world);

  // Check if we can shall use existing behavior models as hypothesis
  if(use_true_behaviors_as_hypothesis_) {
    this->DefineTrueBehaviorsAsHypothesis(observed_world);
  }

  mcts_observed_world->GetEgoAgent()->SetBehaviorModel(ego_behavior_model_);

  const ObservedWorldPtr const_mcts_observed_world =
      std::const_pointer_cast<ObservedWorld>(mcts_observed_world);
  BehaviorMotionPrimitives::MotionIdx num =
      this->ego_behavior_model_->GetNumMotionPrimitives(const_mcts_observed_world);

  // Define initial mcts state
  auto ego_id = const_mcts_observed_world->GetEgoAgentId();
  auto mcts_hypothesis_state_ptr = std::make_shared<MctsStateRiskConstraint>(
                                mcts_observed_world, 
                                false, // terminal
                                num, // num action 
                                prediction_time_span_, 
                                belief_tracker_.sample_current_hypothesis(), // pass hypothesis reference to states
                                behavior_hypotheses_,
                                ego_id,
                                mcts_state_parameters_,
                                belief_tracker_.get_beliefs(),
                                1.0);

  // Belief update for all agents not only filtered
  UpdateBeliefs(observed_world);

  // Update the constraint if it was not already calculated previously during plan
  if(estimate_scenario_risk_ && 
    !initialized_available_risk_ && 
    belief_tracker_.beliefs_initialized()) {
    current_scenario_risk_ = CalculateAvailableScenarioRisk();
    initialized_available_risk_ = true;
  }

  // Do the search
  auto current_mcts_parameters = mcts_parameters_;
  current_mcts_parameters.cost_constrained_statistic.COST_CONSTRAINT = current_scenario_risk_;
  mcts::Mcts<MctsStateHypothesis<MctsStateRiskConstraint>, mcts::CostConstrainedStatistic, mcts::HypothesisStatistic,
             mcts::RandomHeuristic>  mcts_risk_constrained(current_mcts_parameters);
  mcts_risk_constrained.search(*mcts_hypothesis_state_ptr, belief_tracker_);
  auto sampled_policy = mcts_risk_constrained.get_root().get_ego_int_node().greedy_policy(
              0, current_mcts_parameters.cost_constrained_statistic.ACTION_FILTER_FACTOR);
      VLOG(3) << "Constraint: " << current_mcts_parameters.cost_constrained_statistic.COST_CONSTRAINT << ", Action: " << sampled_policy.first << "\n" <<
                mcts_risk_constrained.get_root().get_ego_int_node().print_edge_information(sampled_policy.first) << mcts::CostConstrainedStatistic::print_policy(sampled_policy.second) << "\n"
                << "Expected risk: " << mcts_risk_constrained.get_root().get_ego_int_node().expected_policy_cost(sampled_policy.second);


  // Update the constraint based on policy
  if(initialized_available_risk_ && update_scenario_risk_) {
    current_scenario_risk_ = mcts_risk_constrained.get_root().get_ego_int_node().
                      calc_updated_constraint_based_on_policy(sampled_policy, current_scenario_risk_);
    current_scenario_risk_ = std::min(std::max(0.0, current_scenario_risk_), 1.0);
  }

  // Postprocessing
  if (dump_tree_) {
    std::stringstream filename;
    filename << "tree_dot_file_" << observed_world.GetWorldTime();
    mcts_risk_constrained.printTreeToDotFile(filename.str());
  }

  if(extract_edge_info_) {
    SetLastMctsEdgeInfo(BehaviorUCTBase::ExtractMctsEdgeInfo<mcts::Mcts<MctsStateHypothesis<MctsStateRiskConstraint>, mcts::CostConstrainedStatistic, mcts::HypothesisStatistic,
             mcts::RandomHeuristic>, MctsStateHypothesis<MctsStateRiskConstraint>>(mcts_risk_constrained, max_extraction_depth_));
  }

  const auto& best_action = sampled_policy.first;
  last_motion_idx_ = best_action;
  VLOG(2) << "BehaviorUCTRiskContraint, iterations: " << mcts_risk_constrained.numIterations()
            << ", search time " << mcts_risk_constrained.searchTime()
            << ", best action: " << best_action  << " being " << GetPrimitiveName(best_action);
  VLOG_EVERY_N(3, 3) << belief_tracker_.sprintf();

  // Convert action to a trajectory
  ego_behavior_model_->ActionToBehavior(BehaviorMotionPrimitives::MotionIdx(best_action));
  auto traj = ego_behavior_model_->Plan(delta_time, observed_world);
  this->SetLastTrajectory(traj);
  this->SetLastAction(ego_behavior_model_->GetLastAction());
  this->SetBehaviorStatus(BehaviorStatus::VALID);
  return traj;
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