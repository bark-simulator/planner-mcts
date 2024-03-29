#include "bark_mcts/models/behavior/behavior_uct_hypothesis.hpp"
#define MCTS_EXPECT_TRUE(cond) BARK_EXPECT_TRUE(cond)
#include "mcts/heuristics/random_heuristic.h"
#include "mcts/mcts.h"
#include "mcts/statistics/uct_statistic.h"
#include "mcts/hypothesis/hypothesis_statistic.h"
#include "bark_mcts/models/behavior/mcts_state/mcts_state_hypothesis.hpp"

#include "bark/world/observed_world.hpp"

namespace bark {
namespace models {
namespace behavior {

BehaviorUCTHypothesis::BehaviorUCTHypothesis(const commons::ParamsPtr& params,
                                const std::vector<BehaviorModelPtr>& behavior_hypothesis,
                                const UctHypothesisDebugInfos& hypothesis_debug_infos,
                                const UctBaseDebugInfos& base_debug_infos) :
                                BehaviorUCTHypothesisBase(
                                    params, behavior_hypothesis, hypothesis_debug_infos, base_debug_infos) {}

BehaviorUCTHypothesis::BehaviorUCTHypothesis(const commons::ParamsPtr& params,
                               const std::vector<BehaviorModelPtr>& behavior_hypothesis) :
                               BehaviorUCTHypothesis(params, behavior_hypothesis, UctHypothesisDebugInfos(),
                                      UctBaseDebugInfos()) {}

dynamic::Trajectory BehaviorUCTHypothesis::Plan(
    double min_planning_time, const world::ObservedWorld& observed_world) {
  ObservedWorldPtr mcts_observed_world = BehaviorUCTBase::FilterAgents(observed_world);

  mcts_observed_world->GetEgoAgent()->SetBehaviorModel(ego_behavior_model_);

  // Check if we can shall use existing behavior models as hypothesis
  if(use_true_behaviors_as_hypothesis_) {
    this->DefineTrueBehaviorsAsHypothesis(observed_world);
  }

  const ObservedWorldPtr const_mcts_observed_world =
      std::const_pointer_cast<ObservedWorld>(mcts_observed_world);
  BehaviorMotionPrimitives::MotionIdx num =
      this->ego_behavior_model_->GetNumMotionPrimitives(const_mcts_observed_world);

  // Define initial mcts state
  auto ego_id = const_mcts_observed_world->GetEgoAgentId();
  auto mcts_hypothesis_state_ptr = std::make_shared<MctsStateHypothesis<>>(
                                mcts_observed_world, 
                                false, // terminal
                                num, // num action 
                                1, 
                                belief_tracker_.sample_current_hypothesis(), // pass hypothesis reference to states
                                behavior_hypotheses_,
                                ego_id,
                                mcts_state_parameters_);

  // Belief update for all agents not only filtered
  UpdateBeliefs(observed_world);

  // Now do the search
  mcts::Mcts<MctsStateHypothesis<>, mcts::UctStatistic, mcts::HypothesisStatistic,
             mcts::RandomHeuristic>mcts_hypothesis(mcts_parameters_);
  mcts_hypothesis.search(*mcts_hypothesis_state_ptr, belief_tracker_);
  mcts::ActionIdx best_action = mcts_hypothesis.returnBestAction();
  last_motion_idx_ = best_action;

  SetLastReturnValues(mcts_hypothesis.get_root().get_ego_int_node().get_policy());
  SetLastSolutionTime(mcts_hypothesis.searchTime());

  if (dump_tree_) {
    std::stringstream filename;
    filename << "tree_dot_file_" << observed_world.GetWorldTime();
    mcts_hypothesis.printTreeToDotFile(filename.str());
  }

  if(extract_edge_info_) {
    SetLastMctsEdgeInfo(BehaviorUCTBase::ExtractMctsEdgeInfo<mcts::Mcts<MctsStateHypothesis<>, mcts::UctStatistic, mcts::HypothesisStatistic,
             mcts::RandomHeuristic>, MctsStateHypothesis<>>(mcts_hypothesis, max_extraction_depth_));
  }

  if(extract_state_info_) {
    SetLastMctsStateInfo(BehaviorUCTBase::ExtractMctsStateInfo<mcts::Mcts<MctsStateHypothesis<>, mcts::UctStatistic, mcts::HypothesisStatistic,
             mcts::RandomHeuristic>, MctsStateHypothesis<>>(mcts_hypothesis, max_extraction_depth_));
  }


VLOG(3)   << "Infos: "  <<  mcts_hypothesis.get_root().get_ego_int_node().sprintf();
  VLOG(2) << "BehaviorUCTHypothesis, iterations: " << mcts_hypothesis.numIterations()
            << ", search time " << mcts_hypothesis.searchTime()
            << ", num nodes " << mcts_hypothesis.numNodes()
            << ", best action: " << best_action  << " being " << GetPrimitiveName(best_action);
  VLOG_EVERY_N(3, 3) << belief_tracker_.sprintf();
  SetLastNumIterations(mcts_hypothesis.numIterations());
  SetLastNumNodes(mcts_hypothesis.numNodes());

  // Covert action to a trajectory
  if(constant_action_idx_ < 0) {
    ego_behavior_model_->ActionToBehavior(BehaviorMotionPrimitives::MotionIdx(best_action));
  } else {
    ego_behavior_model_->ActionToBehavior(BehaviorMotionPrimitives::MotionIdx(constant_action_idx_));
  }
  
  auto traj = ego_behavior_model_->Plan(min_planning_time, observed_world);
  SetLastTrajectory(traj);
  SetLastAction(ego_behavior_model_->GetLastAction());
  SetBehaviorStatus(BehaviorStatus::VALID);
  return traj;
}

}  // namespace behavior
}  // namespace models
}  // namespace bark