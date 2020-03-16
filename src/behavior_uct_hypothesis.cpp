#include "src/behavior_uct_hypothesis.hpp"
#include "src/mcts_parameters_from_param_server.hpp"
#define MCTS_EXPECT_TRUE(cond) BARK_EXPECT_TRUE(cond)
#include "mcts/heuristics/random_heuristic.h"
#include "mcts/mcts.h"
#include "mcts/statistics/uct_statistic.h"
#include "mcts/hypothesis/hypothesis_statistic.h"
#include "src/mcts_state_hypothesis.hpp"

#include "modules/models/behavior/constant_velocity/constant_velocity.hpp"
#include "modules/models/behavior/idm/idm_classic.hpp"
#include "modules/models/behavior/motion_primitives/continuous_actions.hpp"
#include "modules/models/dynamic/single_track.hpp"
#include "modules/world/observed_world.hpp"

namespace modules {
namespace models {
namespace behavior {

using modules::world::objects::AgentId;


BehaviorUCTHypothesis::BehaviorUctHypothesis(
    const commons::ParamsPtr& params)
    : BehaviorModel(params),
      mcts_parameters_(models::behavior::MctsParametersFromParamServer(
          GetParams()->AddChild("BehaviorUctSingleAgent"))),
      dump_tree_(GetParams()->AddChild("BehaviorUctSingleAgent")->GetBool(
          "DumpTree",
          "If true, tree is dumped to dot file after planning", false)),
        prediction_time_span_(GetParams()->AddChild("BehaviorUctSingleAgent")
                                        ->AddChild("PredictionSettings")
                                        ->GetReal("TimeSpan",
          "Time in seconds agents are predicted ahead in each expansion and rollout step", 0.5f)),
      belief_tracker_(mcts_parameters_),
      last_mcts_hypothesis_state_() {}


dynamic::Trajectory BehaviorUCTSingleAgentBase::Plan(
    float delta_time, const world::ObservedWorld& observed_world) {
  ObservedWorldPtr mcts_observed_world =
      std::dynamic_pointer_cast<ObservedWorld>(observed_world.Clone());
  // clear behavior models, we set them based on hypothesis in mcts state execute
  auto agents = mcts_observed_world->GetAgents();
  for (const auto& agent : agents) {
    agent.second->SetBehaviorModel(nullptr);
  }

  // Define mcts
  mcts::Mcts<MctsStateHypothesis, mcts::UctStatistic, mcts::HypothesisStatistic,
             mcts::RandomHeuristic>
      mcts(mcts_parameters_);

  const ObservedWorldPtr const_mcts_observed_world =
      std::const_pointer_cast<ObservedWorld>(mcts_observed_world);
  BehaviorMotionPrimitives::MotionIdx num =
      ego_behavior_model_->GetNumMotionPrimitives(const_mcts_observed_world);

  // Define initial mcts state
  auto mcts_hypothesis_state_ptr = std::make_shared<MctsStateHypothesis>(
                                mcts_observed_world, 
                                false, // terminal
                                num, // num action 
                                prediction_time_span_, 
                                belief_tracker_.sample_current_hypothesis(), // pass hypothesis reference to states
                                behavior_hypothesis_,
                                ego_behavior_model_);
  // if this is first call to Plan init belief tracker
  if(!last_mcts_hypothesis_state_) {
    belief_tracker_.belief_update(*mcts_hypothesis_state_ptr, *mcts_hypothesis_state_ptr);
  } else {
    belief_tracker_.belief_update(*mcts_hypothesis_state_ptr, *last_mcts_hypothesis_state_);
  }
  mcts.search(*mcts_hypothesis_state_ptr);
  last_mcts_hypothesis_state_ = mcts_hypothesis_state_ptr;
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

std::vector<AgentId> BehaviorUCTHypothesis::get_agent_id_map (
    const world::ObservedWorld &observed_world) const {
  world::AgentMap agent_map = observed_world.GetOtherAgents();
  std::vector<AgentId> agent_ids(agent_map.size()+1);
  agent_ids[MctsStateHypothesis::ego_agent_idx] =
           observed_world.GetEgoAgent()->GetAgentId();
  size_t i = 1;
  for (const auto &agent : agent_map) {
      agent_ids[i] = agent.first;
      ++i;
    }
  }
  return agent_ids;
}