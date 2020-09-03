// Copyright (c) 2020 Julian Bernhard
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "bark_mcts/models/behavior/behavior_uct_single_agent_base.hpp"
#include "bark_mcts/models/behavior/param_config/mcts_parameters_from_param_server.hpp"
#define MCTS_EXPECT_TRUE(cond) BARK_EXPECT_TRUE(cond)
#include "mcts/heuristics/random_heuristic.h"
#include "mcts/mcts.h"
#include "mcts/statistics/uct_statistic.h"
#include "bark_mcts/models/behavior/mcts_state/mcts_state_single_agent.hpp"

#include "bark/models/behavior/motion_primitives/continuous_actions.hpp"
#include "bark/models/dynamic/single_track.hpp"
#include "bark/world/observed_world.hpp"

namespace bark {
namespace models {
namespace behavior {

using bark::models::dynamic::Input;
using bark::models::dynamic::SingleTrackModel;
using bark::world::ObservedWorldPtr;
using bark::world::prediction::PredictionSettings;

BehaviorUCTSingleAgentBase::BehaviorUCTSingleAgentBase(
    const commons::ParamsPtr& params)
    : BehaviorModel(params->AddChild("BehaviorUctSingleAgent")),
      prediction_settings_(),
      mcts_parameters_(models::behavior::MctsParametersFromParamServer(
          GetParams())),
      dump_tree_(GetParams()->GetBool(
          "DumpTree",
          "If true, tree is dumped to dot file after planning", false)) {}

dynamic::Trajectory BehaviorUCTSingleAgentBase::Plan(
    float delta_time, const world::ObservedWorld& observed_world) {
  ObservedWorldPtr mcts_observed_world =
      std::dynamic_pointer_cast<ObservedWorld>(observed_world.Clone());
  mcts_observed_world->SetupPrediction(prediction_settings_);

  mcts::Mcts<MctsStateSingleAgent, mcts::UctStatistic, mcts::UctStatistic,
             mcts::RandomHeuristic>
      mcts(mcts_parameters_);

  std::shared_ptr<BehaviorMotionPrimitives> ego_model =
      std::dynamic_pointer_cast<BehaviorMotionPrimitives>(
          prediction_settings_.ego_prediction_model_);

  const ObservedWorldPtr const_mcts_observed_world =
      std::const_pointer_cast<ObservedWorld>(mcts_observed_world);
  BehaviorMotionPrimitives::MotionIdx num =
      ego_model->GetNumMotionPrimitives(const_mcts_observed_world);
  MctsStateSingleAgent mcts_state(mcts_observed_world, false, num, delta_time);
  mcts.search(mcts_state);
  mcts::ActionIdx best_action = mcts.returnBestAction();
  SetLastAction(DiscreteAction(best_action));

  if (dump_tree_) {
    std::stringstream filename;
    filename << "tree_dot_file_" << delta_time;
    mcts.printTreeToDotFile(filename.str());
  }

  LOG(INFO) << "BehaviorUCTSingleAgent, iterations: " << mcts.numIterations()
            << ", search time " << mcts.searchTime()
            << ", best action: " << best_action;

  ego_model->ActionToBehavior(BehaviorMotionPrimitives::MotionIdx(best_action));
  auto traj = ego_model->Plan(delta_time, observed_world);
  SetLastTrajectory(traj);
  return traj;
}

}  // namespace behavior
}  // namespace models
}  // namespace bark
