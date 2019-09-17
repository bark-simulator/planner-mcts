// Copyright (c) 2019 fortiss GmbH
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "src/behavior_uct_single_agent.hpp"
#define MCTS_EXPECT_TRUE(cond) BARK_EXPECT_TRUE(cond)
#include "src/mcts_state_single_agent.hpp"
#include "mcts/mcts.h"
#include "mcts/heuristics/random_heuristic.h"
#include "mcts/statistics/uct_statistic.h"

#include "modules/world/observed_world.hpp"
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
    max_num_iterations_(params->get_int("BehaviorUCTSingleAgent::MaxNumIterations", "Maximum number of mcts search iterations", 2000)),
    max_search_time_(params->get_int("BehaviorUCTSingleAgent::MaxSearchTime", "Maximum search time in milliseconds", 2000)),
    random_seed_(params->get_int("BehaviorUCTSingleAgent::RandomSeed", "Random seed applied used during search process", 1000)),
    dump_tree_(params->get_bool("BehaviorUCTSingleAgent::DumpTree", "If true, tree is dumped to dot file after planning", false)),
    discount_factor_(params->get_real("BehaviorUCTSingleAgent::DiscountFactor", "Discount factor used in MDP problem", 0.9)),
    uct_exploration_constant_(params->get_real("BehaviorUCTSingleAgent::UCTExplorationConstant", "Exploration constant of UCT", 0.7)),
    max_search_time_random_heuristic_(params->get_real("BehaviorUCTSingleAgent::MaxSearchTimeRandomHeuristic",
                                                         "Maximum time available for random rollout in milliseconds", 1)),
    max_number_iterations_random_heuristic_(params->get_real("BehaviorUCTSingleAgent::MaxNumIterationsRandomHeuristic",
                                                         "Maximum number of environment steps performed by random heuristic", 0.9)),
    return_lower_bound_(params->get_real("BehaviorUCTSingleAgent::ReturnLowerBound", "Lower return bound used for normalization in UCT Statistic", -1000)),
    return_upper_bound_(params->get_real("BehaviorUCTSingleAgent::ReturnUpperBound", "Upper return bound used for normalization in UCT Statistic", 100)) {

    // Setup prediction models for ego agent and other agents
    DynamicModelPtr dyn_model(new SingleTrackModel(params));
    BehaviorModelPtr ego_prediction_model(new BehaviorMotionPrimitives(dyn_model, params));

    auto input_list = params->get_listlist_float("BehaviorUCTSingleAgent::MotionPrimitiveInputs", "A list of pairs with "
                                                 "acceleration and steering angle to define the motion primitives",
                                                 {{0,0}, {5,0}, {0,-1}, {0, 1}, {-3,0}});
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

    update_mcts_parameters();

    ObservedWorldPtr mcts_observed_world(observed_world.Clone());
    mcts_observed_world->SetupPrediction(prediction_settings_);

    mcts::RandomGenerator::random_generator_ = std::mt19937(random_seed_);
    mcts::Mcts<MctsStateSingleAgent, mcts::UctStatistic, mcts::UctStatistic, mcts::RandomHeuristic> mcts;

    auto ego_model = std::dynamic_pointer_cast<BehaviorMotionPrimitives>(prediction_settings_.ego_prediction_model_);
    MctsStateSingleAgent mcts_state(mcts_observed_world, false, ego_model->GetNumMotionPrimitives(), delta_time);
    mcts.search(mcts_state, max_search_time_, max_num_iterations_);
    mcts::ActionIdx best_action = mcts.returnBestAction();
    set_last_action(DiscreteAction(best_action));

    if(dump_tree_) {
        std::stringstream filename;
        filename << "tree_dot_file_" << delta_time;
        mcts.printTreeToDotFile(filename.str());
    }

    ego_model->ActionToBehavior(BehaviorMotionPrimitives::MotionIdx(best_action));
    auto traj = ego_model->Plan(delta_time, observed_world);
    set_last_trajectory(traj);
    return traj;
}

void BehaviorUCTSingleAgent::update_mcts_parameters() {
    // Todo: Change static to instance-based parameters in MCTS (otherwise problems if more than one agent uses mcts with different parameters)
    mcts::MctsParameters::DISCOUNT_FACTOR = discount_factor_;
    mcts::MctsParameters::EXPLORATION_CONSTANT = uct_exploration_constant_;
    mcts::MctsParameters::MAX_SEARCH_TIME_RANDOM_HEURISTIC = max_search_time_random_heuristic_;
    mcts::MctsParameters::MAX_NUMBER_OF_ITERATIONS_RANDOM_HEURISTIC = max_number_iterations_random_heuristic_;
    mcts::MctsParameters::LOWER_BOUND = return_lower_bound_;
    mcts::MctsParameters::UPPER_BOUND = return_upper_bound_;
}

}  // namespace behavior
}  // namespace models
}  // namespace modules