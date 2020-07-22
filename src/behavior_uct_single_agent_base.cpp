// Copyright (c) 2019 fortiss GmbH
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "src/behavior_uct_single_agent_base.hpp"
#include "src/mcts_parameters_from_param_server.hpp"
#define MCTS_EXPECT_TRUE(cond) BARK_EXPECT_TRUE(cond)
#include "mcts/heuristics/random_heuristic.h"
#include "mcts/mcts.h"
#include "mcts/statistics/uct_statistic.h"
#include "src/mcts_state_single_agent.hpp"
#include "src/domain_heuristic.hpp"
#include "src/NN_heuristic.hpp"

#include "modules/models/behavior/constant_velocity/constant_velocity.hpp"
#include "modules/models/behavior/idm/idm_classic.hpp"
#include "modules/models/behavior/motion_primitives/continuous_actions.hpp"
#include "modules/models/dynamic/single_track.hpp"
#include "modules/world/observed_world.hpp"

namespace modules {
namespace models {
namespace behavior {

using modules::models::dynamic::Input;
using modules::models::dynamic::SingleTrackModel;
using modules::world::ObservedWorldPtr;
using modules::world::prediction::PredictionSettings;

BehaviorUCTSingleAgentBase::BehaviorUCTSingleAgentBase(
    const commons::ParamsPtr& params)
    : BehaviorModel(params),
      prediction_settings_(),
      mcts_parameters_(models::behavior::MctsParametersFromParamServer(
          GetParams()->AddChild("BehaviorUctSingleAgent"))),
      dump_tree_(GetParams()->AddChild("BehaviorUctSingleAgent")->GetBool(
          "DumpTree",
          "If true, tree is dumped to dot file after planning", false)),
      random_heuristic_(GetParams()->AddChild("BehaviorUctSingleAgent")->GetBool(
          "UseRandomHeuristic", "True if random heuristic shall be used, otherwise domain heuristic is applied", false)),
      nn_heuristic_(GetParams()->AddChild("BehaviorUctSingleAgent")->GetBool(
          "UseNNHeuristic", "True if nn heuristic shall be used, otherwise domain heuristic is applied", false)),
      prediction_time_span_(GetParams()->AddChild("BehaviorUctSingleAgent")
                                        ->AddChild("PredictionSettings")
                                        ->GetReal("TimeSpan",
          "Time in seconds agents are predicted ahead in each expansion and rollout step", 0.5f)) {
              
          }

dynamic::Trajectory BehaviorUCTSingleAgentBase::Plan(
    float delta_time, const world::ObservedWorld& observed_world) {
  ObservedWorldPtr mcts_observed_world =
      std::dynamic_pointer_cast<ObservedWorld>(observed_world.Clone());
  mcts_observed_world->SetupPrediction(prediction_settings_);

  mcts::Mcts<MctsStateSingleAgent, mcts::UctStatistic, mcts::UctStatistic,
             mcts::RandomHeuristic> mcts_random (mcts_parameters_);
  mcts::Mcts<MctsStateSingleAgent, mcts::UctStatistic, mcts::UctStatistic,
             DomainHeuristic> mcts_domain(mcts_parameters_);
  mcts::Mcts<MctsStateSingleAgent, mcts::UctStatistic, mcts::UctStatistic,
             NNHeuristic> mcts_nn(mcts_parameters_);

  std::shared_ptr<BehaviorMotionPrimitives> ego_model =
      std::dynamic_pointer_cast<BehaviorMotionPrimitives>(
          prediction_settings_.ego_prediction_model_);

  const ObservedWorldPtr const_mcts_observed_world =
      std::const_pointer_cast<ObservedWorld>(mcts_observed_world);
  BehaviorMotionPrimitives::MotionIdx num =
      ego_model->GetNumMotionPrimitives(const_mcts_observed_world);

  MctsStateSingleAgent mcts_state(mcts_observed_world, false, num, prediction_time_span_);
  
 
  mcts::ActionIdx best_action;
  unsigned num_iterations;
  unsigned search_time;
  
  if((random_heuristic_)&&(nn_heuristic_)) {
      LOG(ERROR) << "Can't use random_heuristic and nn_heuristic at same time.";
      throw;
      } else if((random_heuristic_)&&!(nn_heuristic_)) {
          NNHeuristic::InitializeModelLoader();
          NNHeuristic::InitializeObserver();
          mcts_random.search(mcts_state);
          best_action = mcts_random.returnBestAction();
          num_iterations = mcts_random.numIterations();
          search_time = mcts_random.searchTime();
          if (dump_tree_) {
              std::stringstream filename;
              filename << "tree_dot_file_" << delta_time;
              mcts_random.printTreeToDotFile(filename.str());}
      } else if(!(random_heuristic_)&&(nn_heuristic_)) {
          mcts_nn.search(mcts_state);
          best_action = mcts_nn.returnBestAction();
          num_iterations = mcts_nn.numIterations();
          search_time = mcts_nn.searchTime();
          if (dump_tree_) {
              std::stringstream filename;
              filename << "tree_dot_file_" << delta_time;
              mcts_nn.printTreeToDotFile(filename.str());}
      } else{
          mcts_domain.search(mcts_state);
          best_action = mcts_domain.returnBestAction();
          num_iterations = mcts_domain.numIterations();
          search_time = mcts_domain.searchTime();
          if (dump_tree_) {
              std::stringstream filename;
              filename << "tree_dot_file_" << delta_time;
              mcts_domain.printTreeToDotFile(filename.str());}
      }
            
  

  SetLastAction(DiscreteAction(best_action));
  LOG(INFO) << "BehaviorUCTSingleAgent, iterations: " << num_iterations
            << ", search time " << search_time
            << ", best action: " << best_action;

  ego_model->ActionToBehavior(BehaviorMotionPrimitives::MotionIdx(best_action));
  auto traj = ego_model->Plan(delta_time, observed_world);
  SetLastTrajectory(traj);
  SetBehaviorStatus(BehaviorStatus::VALID);
  return traj;
}

}  // namespace behavior
}  // namespace models
}  // namespace modules
