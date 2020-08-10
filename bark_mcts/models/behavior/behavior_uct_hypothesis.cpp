#include "bark_mcts/models/behavior/behavior_uct_hypothesis.hpp"
#define MCTS_EXPECT_TRUE(cond) BARK_EXPECT_TRUE(cond)
#include "mcts/heuristics/random_heuristic.h"
#include "mcts/mcts.h"
#include "mcts/statistics/uct_statistic.h"
#include "mcts/hypothesis/hypothesis_statistic.h"
#include "bark_mcts/models/behavior/mcts_state/mcts_state_hypothesis.hpp"

#include "bark/models/behavior/constant_velocity/constant_velocity.hpp"
#include "bark/models/behavior/idm/idm_classic.hpp"
#include "bark/models/behavior/motion_primitives/continuous_actions.hpp"
#include "bark/models/dynamic/single_track.hpp"
#include "bark/world/observed_world.hpp"

namespace bark {
namespace models {
namespace behavior {

using bark::world::objects::AgentId;

BehaviorUCTHypothesis::BehaviorUCTHypothesis(const commons::ParamsPtr& params,
                                const std::vector<BehaviorModelPtr>& behavior_hypothesis) :
                                BehaviorUCTHypothesisBase(params, behavior_hypothesis) {}

dynamic::Trajectory BehaviorUCTHypothesis::Plan(
    float delta_time, const world::ObservedWorld& observed_world) {
  ObservedWorldPtr mcts_observed_world =
      std::dynamic_pointer_cast<ObservedWorld>(observed_world.Clone());

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
                                prediction_time_span_, 
                                belief_tracker_.sample_current_hypothesis(), // pass hypothesis reference to states
                                behavior_hypotheses_,
                                ego_behavior_model_,
                                ego_id,
                                mcts_state_parameters_);

  // Belief update only required, if we do not use true behaviors as hypothesis
  if(!use_true_behaviors_as_hypothesis_) {
    // if this is first call to Plan init belief tracker
    if(!last_mcts_hypothesis_state_) {
      belief_tracker_.belief_update(*mcts_hypothesis_state_ptr, *mcts_hypothesis_state_ptr);
    } else {
      belief_tracker_.belief_update(*last_mcts_hypothesis_state_, *mcts_hypothesis_state_ptr);
    }
  }

  // Now do the search
  mcts::Mcts<MctsStateHypothesis<>, mcts::UctStatistic, mcts::HypothesisStatistic,
             mcts::RandomHeuristic>mcts_hypothesis(mcts_parameters_);
  mcts_hypothesis.search(*mcts_hypothesis_state_ptr, belief_tracker_);
  last_mcts_hypothesis_state_ = mcts_hypothesis_state_ptr;
  mcts::ActionIdx best_action = mcts_hypothesis.returnBestAction();
  this->SetLastAction(DiscreteAction(best_action));

  if (dump_tree_) {
    std::stringstream filename;
    filename << "tree_dot_file_" << observed_world.GetWorldTime();
    mcts_hypothesis.printTreeToDotFile(filename.str());
  }

  VLOG(2) << "BehaviorUCTHypothesis, iterations: " << mcts_hypothesis.numIterations()
            << ", search time " << mcts_hypothesis.searchTime()
            << ", best action: " << best_action;
  VLOG_EVERY_N(3, 3) << belief_tracker_.sprintf();

  // Covert action to a trajectory
  ego_behavior_model_->ActionToBehavior(BehaviorMotionPrimitives::MotionIdx(best_action));
  auto traj = ego_behavior_model_->Plan(delta_time, observed_world);
  SetLastTrajectory(traj);
  SetLastAction(ego_behavior_model_->GetLastAction());
  SetBehaviorStatus(BehaviorStatus::VALID);
  return traj;
}

}  // namespace behavior
}  // namespace models
}  // namespace bark