// Copyright (c) 2019 fortiss GmbH
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "src/behavior_uct_single_agent.hpp"
#include "src/mcts_parameters_from_param_server.hpp"
#define MCTS_EXPECT_TRUE(cond) BARK_EXPECT_TRUE(cond)
#include "src/mcts_state_single_agent.hpp"
#include "mcts/mcts.h"
#include "mcts/heuristics/random_heuristic.h"
#include "mcts/statistics/uct_statistic.h"

#include "modules/world/observed_world.hpp"
#include "modules/models/behavior/motion_primitives/continuous_actions.hpp"
#include "modules/models/behavior/constant_velocity/constant_velocity.hpp"
#include "modules/models/behavior/idm/idm_classic.hpp"
#include "modules/models/dynamic/single_track.hpp"

namespace modules {
namespace models {
namespace behavior {

using modules::world::prediction::PredictionSettings;
using modules::models::dynamic::SingleTrackModel;
using modules::models::dynamic::Input;
using modules::world::ObservedWorldPtr;

BehaviorUCTSingleAgent::BehaviorUCTSingleAgent(const commons::ParamsPtr& params) :
    BehaviorModel(params),
    prediction_settings_(),
    mcts_parameters_(models::behavior::MctsParametersFromParamServer(params->AddChild("BehaviorUctSingleAgent"))),
    dump_tree_(params->GetBool("BehaviorUctSingleAgent::DumpTree", "If true, tree is dumped to dot file after planning", false)) {

    // Setup prediction models for ego agent and other agents
    DynamicModelPtr dyn_model(new SingleTrackModel(params));
    BehaviorModelPtr ego_prediction_model(new BehaviorMPContinuousActions(dyn_model, params));

    auto input_list = params->GetListListFloat("BehaviorUctSingleAgent::MotionPrimitiveInputs", "A list of pairs with "
                                                 "acceleration and steering angle to define the motion primitives",
                                                 {{0,0}, {5,0}, {0,-1}, {0, 1}, {-3,0}});
    for(const auto& input : input_list) {
        BARK_EXPECT_TRUE(input.size() == 2);
        Input u(2);  u << input[0], input[1];
        BehaviorMotionPrimitives::MotionIdx idx = std::dynamic_pointer_cast<BehaviorMPContinuousActions>(ego_prediction_model)->AddMotionPrimitive(u);
    }
    
    BehaviorModelPtr others_prediction_model(new BehaviorIDMClassic(params));
    prediction_settings_ = PredictionSettings(ego_prediction_model, others_prediction_model);
}

dynamic::Trajectory BehaviorUCTSingleAgent::Plan(
    float delta_time,
    const world::ObservedWorld& observed_world) {

    ObservedWorldPtr mcts_observed_world = std::dynamic_pointer_cast<ObservedWorld>(observed_world.Clone());
    mcts_observed_world->SetupPrediction(prediction_settings_);

    mcts::Mcts<MctsStateSingleAgent, mcts::UctStatistic, mcts::UctStatistic, mcts::RandomHeuristic> mcts(mcts_parameters_);

    std::shared_ptr<BehaviorMotionPrimitives> ego_model = std::dynamic_pointer_cast<BehaviorMotionPrimitives>(prediction_settings_.ego_prediction_model_);
    
    const ObservedWorldPtr const_mcts_observed_world = std::const_pointer_cast<ObservedWorld>(mcts_observed_world);
    BehaviorMotionPrimitives::MotionIdx num = ego_model->GetNumMotionPrimitives(const_mcts_observed_world);
    MctsStateSingleAgent mcts_state(mcts_observed_world, false, num, delta_time);
    mcts.search(mcts_state);
    mcts::ActionIdx best_action = mcts.returnBestAction();
    SetLastAction(DiscreteAction(best_action));

    if(dump_tree_) {
        std::stringstream filename;
        filename << "tree_dot_file_" << delta_time;
        mcts.printTreeToDotFile(filename.str());
    }

    LOG(INFO) << "BehaviorUCTSingleAgent, iterations: " << mcts.numIterations() << ", search time " << mcts.searchTime() << ", best action: " << best_action;

    ego_model->ActionToBehavior(BehaviorMotionPrimitives::MotionIdx(best_action));
    auto traj = ego_model->Plan(delta_time, observed_world);
    SetLastTrajectory(traj);
    return traj;
}

}  // namespace behavior
}  // namespace models
}  // namespace modules
