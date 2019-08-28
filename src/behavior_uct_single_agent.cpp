// Copyright (c) 2019 fortiss GmbH
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "src/behavior_uct_single_agent.hpp"
#include "src/single_agent_mcts_state.hpp"
#include "mcts/mcts.h"
#include "mcts/heuristics/random_heuristic.h"
#include "mcts/statistics/uct_statistic.h"

#include "modules/models/behavior/motion_primitives/motion_primitives.hpp"
#include "modules/models/behavior/constant_velocity/constant_velocity.hpp"
#include "modules/models/dynamic/single_track.hpp"


namespace modules {
namespace models {
namespace behavior {

using modules::world::prediction::PredictionSettings;
using modules::models::dynamic::SingleTrackModel;
using modules::models::dynamic::Input;
using modules::world::ObservedWorldPtr;

BehaviorUCTSingleAgent::BehaviorUCTSingleAgent(commons::Params *params) :
    BehaviorModel(params),
    prediction_settings_(),
    max_num_iterations_(params->get_int("BehaviorUCTSingleAgent::MaxNumIterations", "Maximum number of mcts search iterations", 5000)),
    max_search_time_(params->get_int("BehaviorUCTSingleAgent::MaxSearchTime", "Maximum search time in milliseconds", 1000)),
    random_seed_(params->get_int("BehaviorUCTSingleAgent::RandomSeed", "Random seed applied used during search process", 1000)) {

    // Setup prediction models for ego agent and other agents
    DynamicModelPtr dyn_model(new SingleTrackModel());
    BehaviorModelPtr ego_prediction_model(new BehaviorMotionPrimitives(dyn_model, params));

    auto input_list = params->get_listlist_float("BehaviorUCTSingleAgent::MotionPrimitiveInputs", "A list of pairs with "
                                                 "acceleration and steering angle to define the motion primitives",
                                                 {{0,0}, {5,0}, {0,-1}, {0, 1}});
    for(const auto& input : input_list) {
        BARK_EXPECT_TRUE(input.size() == 2);
        Input u(2);  u << input[0], input[1];
        BehaviorMotionPrimitives::MotionIdx idx = std::dynamic_pointer_cast<BehaviorMotionPrimitives>(ego_prediction_model)->AddMotionPrimitive(u);
    }
    
    BehaviorModelPtr others_prediction_model(new BehaviorConstantVelocity(params));
    prediction_settings_ = PredictionSettings(ego_prediction_model, others_prediction_model);
}

dynamic::Trajectory BehaviorUCTSingleAgent::Plan(
    float delta_time,
    const world::ObservedWorld& observed_world) {

    ObservedWorldPtr mcts_observed_world(observed_world.Clone());
    mcts_observed_world->SetupPrediction(prediction_settings_);

    mcts::RandomGenerator::random_generator_ = std::mt19937(random_seed_);
    mcts::Mcts<SingleAgentMCTSState, mcts::UctStatistic, mcts::UctStatistic, mcts::RandomHeuristic> mcts;

    auto ego_model = std::dynamic_pointer_cast<BehaviorMotionPrimitives>(prediction_settings_.ego_prediction_model_);
    SingleAgentMCTSState mcts_state(mcts_observed_world, false, ego_model->GetNumMotionPrimitives(), delta_time);
    mcts.search(mcts_state, max_search_time_, max_num_iterations_);
    mcts::ActionIdx best_action = mcts.returnBestAction();

    ego_model->ActionToBehavior(BehaviorMotionPrimitives::MotionIdx(best_action));
    return ego_model->Plan(delta_time, observed_world);
}

}  // namespace behavior
}  // namespace models
}  // namespace modules