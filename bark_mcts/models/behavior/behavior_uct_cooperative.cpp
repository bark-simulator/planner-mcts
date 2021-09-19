#include "bark_mcts/models/behavior/behavior_uct_cooperative.hpp"
#define MCTS_EXPECT_TRUE(cond) BARK_EXPECT_TRUE(cond)
#include "mcts/heuristics/random_heuristic.h"
#include "mcts/mcts.h"
#include "mcts/statistics/uct_statistic.h"
#include "mcts/hypothesis/hypothesis_statistic.h"
#include "bark_mcts/models/behavior/mcts_state/mcts_state_cooperative.hpp"

#include "bark/world/observed_world.hpp"

namespace bark {
namespace models {
namespace behavior {

BehaviorUCTCooperative::BehaviorUCTCooperative(const commons::ParamsPtr& params) :
                                BehaviorUCTBase(params) {}

dynamic::Trajectory BehaviorUCTCooperative::Plan(
    double min_planning_time, const world::ObservedWorld& observed_world) {
  ObservedWorldPtr mcts_observed_world = BehaviorUCTBase::FilterAgents(observed_world);
  for ( auto& agent : mcts_observed_world->GetAgents()) {
    agent.second->SetBehaviorModel(ego_behavior_model_);
  }

  const ObservedWorldPtr const_mcts_observed_world =
      std::const_pointer_cast<ObservedWorld>(mcts_observed_world);
  BehaviorMotionPrimitives::MotionIdx num =
      this->ego_behavior_model_->GetNumMotionPrimitives(const_mcts_observed_world);

  // Define initial mcts state
  auto ego_id = const_mcts_observed_world->GetEgoAgentId();
  auto mcts_cooperative_state_ptr = std::make_shared<MctsStateCooperative>(
                                mcts_observed_world, 
                                false, // terminal
                                num, // num action 
                                1, 
                                ego_id,
                                mcts_state_parameters_);

  // Now do the search
  mcts::Mcts<MctsStateCooperative, mcts::UctStatistic, mcts::UctStatistic,
             mcts::RandomHeuristic> mcts_cooperative(mcts_parameters_);
  mcts_cooperative.search(*mcts_cooperative_state_ptr);
  mcts::ActionIdx best_action = mcts_cooperative.returnBestAction();
  last_motion_idx_ = best_action;
  SetLastReturnValues(mcts_cooperative.get_root().get_ego_int_node().get_policy());
  SetLastSolutionTime(mcts_cooperative.searchTime());

  if (dump_tree_) {
    std::stringstream filename;
    filename << "tree_dot_file_" << observed_world.GetWorldTime();
    mcts_cooperative.printTreeToDotFile(filename.str());
  }

  if(extract_edge_info_) {
    SetLastMctsEdgeInfo(BehaviorUCTBase::ExtractMctsEdgeInfo<mcts::Mcts<MctsStateCooperative, mcts::UctStatistic, mcts::UctStatistic,
             mcts::RandomHeuristic>, MctsStateCooperative>(mcts_cooperative, max_extraction_depth_));
  }

  if(extract_state_info_) {
    SetLastMctsStateInfo(BehaviorUCTBase::ExtractMctsStateInfo<mcts::Mcts<MctsStateCooperative, mcts::UctStatistic, mcts::UctStatistic,
             mcts::RandomHeuristic>, MctsStateCooperative>(mcts_cooperative, max_extraction_depth_));
  }

  VLOG(2) << "BehaviorUCTCooperative, iterations: " << mcts_cooperative.numIterations()
            << ", search time " << mcts_cooperative.searchTime()
            << ", num nodes " << mcts_cooperative.numNodes()
            << ", best action: " << best_action  << " being " << GetPrimitiveName(best_action);

  // Convert action to a trajectory
  ego_behavior_model_->ActionToBehavior(BehaviorMotionPrimitives::MotionIdx(best_action));
  auto traj = ego_behavior_model_->Plan(min_planning_time, observed_world);
  SetLastTrajectory(traj);
  SetLastAction(ego_behavior_model_->GetLastAction());
  SetBehaviorStatus(BehaviorStatus::VALID);
  return traj;
}

}  // namespace behavior
}  // namespace models
}  // namespace bark