// Copyright (c) 2020 Julian Bernhard
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef MODULES_MODELS_BEHAVIOR_BEHAVIOR_UCT_RISK_CONSTRAINT_HPP_
#define MODULES_MODELS_BEHAVIOR_BEHAVIOR_UCT_RISK_CONSTRAINT_HPP_

#include <memory>
#include "bark/commons/util/util.hpp"
#define MCTS_EXPECT_TRUE(cond) BARK_EXPECT_TRUE(cond)
#include "mcts/cost_constrained/cost_constrained_statistic.h"
#include "bark_mcts/models/behavior/behavior_uct_hypothesis_base.hpp"
#include "bark_mcts/models/behavior/mcts_state/mcts_state_risk_constraint.hpp"
#include "bark_mcts/models/behavior/risk_calculation/scenario_risk_function.hpp"

#include "mcts/heuristics/random_heuristic.h"
#include "mcts/statistics/uct_statistic.h"
#include "mcts/hypothesis/hypothesis_statistic.h"

#include "mcts/hypothesis/hypothesis_belief_tracker.h"

#include "bark/world/observed_world.hpp"

namespace bark {
namespace world {
class ObservedWorld;
}
namespace models {
namespace behavior {

typedef std::pair<mcts::ActionIdx, mcts::Policy> PolicySampled;

struct UctRiskConstraintDebugInfos {
  std::vector<mcts::Cost> GetLastScenarioRisk() const { return last_scenario_risk_; }
  void SetLastScenarioRisk(const std::vector<mcts::Cost>& risk) { last_scenario_risk_ = risk; }

  std::vector<mcts::Cost> GetLastExpectedRisk() const { return last_expected_risk_; }
  void SetLastExpectedRisk(const std::vector<mcts::Cost>& risk) { last_expected_risk_ = risk; }

  PolicySampled GetLastPolicySampled() const { return last_policy_sampled_; }
  void SetLastPolicySampled(const PolicySampled& policy_sampled) { last_policy_sampled_ = policy_sampled; }
  
  void SetLastCostValues(const std::string cost_name, const mcts::Policy& cost_values) { last_cost_values_[cost_name] = cost_values;}
  std::unordered_map<std::string, mcts::Policy> GetLastCostValues() const { return last_cost_values_; }

  PolicySampled last_policy_sampled_;
  std::vector<mcts::Cost> last_expected_risk_;
  std::unordered_map<std::string, mcts::Policy> last_cost_values_;
  std::vector<mcts::Cost> last_scenario_risk_;
};

class BehaviorUCTRiskConstraint : public BehaviorUCTHypothesisBase<MctsStateRiskConstraint>,
                                  public UctRiskConstraintDebugInfos{
 public:
  explicit BehaviorUCTRiskConstraint(const commons::ParamsPtr& params,
                                const std::vector<BehaviorModelPtr>& behavior_hypothesis,
                                const risk_calculation::ScenarioRiskFunctionPtr& scenario_risk_function,
                                const UctHypothesisDebugInfos& hypothesis_debug_infos,
                                const UctRiskConstraintDebugInfos& risk_constraint_debug_infos,
                                const UctBaseDebugInfos& base_debug_infos);

  explicit BehaviorUCTRiskConstraint(const commons::ParamsPtr& params,
                                const std::vector<BehaviorModelPtr>& behavior_hypothesis,
                                const risk_calculation::ScenarioRiskFunctionPtr& scenario_risk_function);

  virtual ~BehaviorUCTRiskConstraint() {}

  virtual Trajectory Plan(double min_planning_time,
                          const world::ObservedWorld& observed_world);


  virtual std::shared_ptr<BehaviorModel> Clone() const;

  risk_calculation::ScenarioRiskFunctionPtr GetScenarioRiskFunction() const { return scenario_risk_function_; }

  protected:
    template<class S, class SE, class SO, class H>
    Trajectory PlanWithMcts(double min_planning_time,
                          const world::ObservedWorld& observed_world);

    virtual void InitializeHeuristic(void* mcts) const {}

    mcts::Cost CalculateAvailableScenarioRisk() const;

    std::vector<mcts::Cost> default_available_risk_;
    std::vector<mcts::Cost> current_scenario_risk_;
    bool estimate_scenario_risk_; // Should scenario risk be estimated from scenario risk function
    bool initialized_available_risk_; // Was scenario risk initialized from scenario risk function after belief was initialized
    bool update_scenario_risk_; // Should scenario risk be updated during scenario execution based on policy and executed actions
    risk_calculation::ScenarioRiskFunctionPtr scenario_risk_function_;
};

inline std::shared_ptr<BehaviorModel> BehaviorUCTRiskConstraint::Clone() const {
  std::shared_ptr<BehaviorUCTRiskConstraint> model_ptr =
      std::make_shared<BehaviorUCTRiskConstraint>(*this);
  return model_ptr;
}


template<class S, class SE, class SO, class H>
Trajectory BehaviorUCTRiskConstraint::PlanWithMcts(double min_planning_time,
                      const world::ObservedWorld& observed_world) {
  ObservedWorldPtr mcts_observed_world = BehaviorUCTBase::FilterAgents(observed_world);
  using mcts::operator<<;
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
                                1, 
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
    // Assuming first risk is to be estimated second collision risk
    current_scenario_risk_[0] = CalculateAvailableScenarioRisk();
    initialized_available_risk_ = true;
  }

  // Do the search
  auto current_mcts_parameters = mcts_parameters_;
  current_mcts_parameters.cost_constrained_statistic.COST_CONSTRAINTS = current_scenario_risk_;
  mcts::Mcts<S, SE, SO,
             H>  mcts_risk_constrained(current_mcts_parameters);
  InitializeHeuristic(&mcts_risk_constrained);
  mcts_risk_constrained.search(*mcts_hypothesis_state_ptr, belief_tracker_);
  auto sampled_policy = mcts_risk_constrained.get_root().get_ego_int_node().greedy_policy(
              0, current_mcts_parameters.cost_constrained_statistic.ACTION_FILTER_FACTOR, false);
  auto expected_risk = mcts_risk_constrained.get_root().get_ego_int_node().expected_policy_cost(sampled_policy.second);
  VLOG(3) << "Constraint: " << current_mcts_parameters.cost_constrained_statistic.COST_CONSTRAINTS << ", Action: " << sampled_policy.first << "\n" <<
            mcts_risk_constrained.get_root().get_ego_int_node().print_edge_information(sampled_policy.first) << SE::print_policy(sampled_policy.second) << "\n"
            << "Expected risk: " << expected_risk;

  SetLastPolicySampled(sampled_policy);
  SetLastExpectedRisk(expected_risk);
  SetLastReturnValues(mcts_risk_constrained.get_root().get_ego_int_node().get_reward_statistic().get_policy());
  SetLastSolutionTime(mcts_risk_constrained.searchTime());
  
  if (mcts_hypothesis_state_ptr->get_num_costs() == 2) {
    SetLastCostValues("collision", mcts_risk_constrained.get_root().get_ego_int_node().get_cost_statistic(1).get_policy());
    SetLastCostValues("envelope", mcts_risk_constrained.get_root().get_ego_int_node().get_cost_statistic(0).get_policy());
  } else {
    SetLastCostValues("cost", mcts_risk_constrained.get_root().get_ego_int_node().get_cost_statistic(0).get_policy());
  }

  
  SetLastScenarioRisk(current_scenario_risk_);
  // Update the constraint based on policy
  if(initialized_available_risk_ && update_scenario_risk_) {
    current_scenario_risk_ = mcts_risk_constrained.get_root().get_ego_int_node().
                      calc_updated_constraints_based_on_policy(sampled_policy, current_scenario_risk_);
    if(current_scenario_risk_.size() > 1) {
          current_scenario_risk_[1] = default_available_risk_.at(1);
    }
    for (auto& risk : current_scenario_risk_) {
      risk = std::min(std::max(0.0, risk), 1.0);
    }
  }

  // Postprocessing
  if (dump_tree_) {
    std::stringstream filename;
    filename << "tree_dot_file_" << observed_world.GetWorldTime();
    mcts_risk_constrained.printTreeToDotFile(filename.str());
  }

  if(extract_edge_info_) {
    SetLastMctsEdgeInfo(BehaviorUCTBase::ExtractMctsEdgeInfo<mcts::Mcts<S, SE, SO,
             H>, S>(mcts_risk_constrained, max_extraction_depth_));
  }

  if(extract_state_info_) {
    SetLastMctsStateInfo(BehaviorUCTBase::ExtractMctsStateInfo<mcts::Mcts<S, SE, SO,
             H>, S>(mcts_risk_constrained, max_extraction_depth_));
  }

  const auto& best_action = sampled_policy.first;
  last_motion_idx_ = best_action;
  VLOG(2) << "BehaviorUCTRiskContraint, iterations: " << mcts_risk_constrained.numIterations()
            << ", search time " << mcts_risk_constrained.searchTime()
            << ", num nodes " << mcts_risk_constrained.numNodes()
            << ", best action: " << best_action  << " being " << GetPrimitiveName(best_action);
  VLOG_EVERY_N(3, 3) << belief_tracker_.sprintf();

  // Convert action to a trajectory
  ego_behavior_model_->ActionToBehavior(BehaviorMotionPrimitives::MotionIdx(best_action));
  auto traj = ego_behavior_model_->Plan(min_planning_time, observed_world);
  this->SetLastTrajectory(traj);
  this->SetLastAction(ego_behavior_model_->GetLastAction());
  this->SetBehaviorStatus(BehaviorStatus::VALID);
  return traj;
}

}  // namespace behavior
}  // namespace models
}  // namespace bark

#endif  // MODULES_MODELS_BEHAVIOR_BEHAVIOR_UCT_RISK_CONSTRAINT_HPP_