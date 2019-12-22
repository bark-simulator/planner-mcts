// Copyright (c) 2019 fortiss GmbH
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "src/mcts_state_single_agent.hpp"

#include "modules/world/observed_world.hpp"
#include "modules/world/evaluation/evaluator_collision_driving_corridor.hpp"
#include "modules/world/evaluation/evaluator_collision_ego_agent.hpp"
#include "modules/world/evaluation/evaluator_goal_reached.hpp"



namespace modules {
namespace models {
namespace behavior {

using modules::world::evaluation::EvaluatorCollisionDrivingCorridor;
using modules::world::evaluation::EvaluatorGoalReached;
using modules::world::evaluation::EvaluatorCollisionEgoAgent;
using modules::world::ObservedWorldPtr;

MctsStateSingleAgent::MctsStateSingleAgent(const ObservedWorldPtr& observed_world, bool is_terminal_state,
                                           const mcts::ActionIdx& num_ego_actions,
                                           const float& prediction_time_span) : 
                                        observed_world_(observed_world),
                                        is_terminal_state_(is_terminal_state),
                                        num_ego_actions_(num_ego_actions),
                                        prediction_time_span_(prediction_time_span) {}

std::shared_ptr<MctsStateSingleAgent> MctsStateSingleAgent::clone() const {
    return std::make_shared<MctsStateSingleAgent>(std::dynamic_pointer_cast<world::ObservedWorld>(observed_world_->Clone()),
                             is_terminal_state_, num_ego_actions_, prediction_time_span_);
}

std::shared_ptr<MctsStateSingleAgent> MctsStateSingleAgent::execute(const mcts::JointAction& joint_action,
                                                                    std::vector< mcts::Reward>& rewards ) const {
    BARK_EXPECT_TRUE(!is_terminal());

    // TODO: parameter for prediction time span
    auto predicted_world = std::dynamic_pointer_cast<world::ObservedWorld>(observed_world_->Predict(prediction_time_span_,
                                 DiscreteAction(joint_action[MctsStateSingleAgent::ego_agent_idx])));

    bool collision_corridor = false;
    bool collision_ego = false;
    bool goal_reached = false;
    bool out_of_map = false;

    if(predicted_world->get_ego_agent()) {
        auto ego_state_x = predicted_world->current_ego_state()[dynamic::StateDefinition::X_POSITION];

        // TODO: allow for separate configuration options ------
        auto evaluator_collision_corridor = EvaluatorCollisionDrivingCorridor();
        auto evaluator_collision_ego = EvaluatorCollisionEgoAgent(predicted_world->get_ego_agent()->get_agent_id());
        auto evaluator_goal_reached = EvaluatorGoalReached(predicted_world->get_ego_agent()->get_agent_id());
        
        collision_corridor = boost::get<bool>(evaluator_collision_corridor.Evaluate(*predicted_world));
        collision_ego = boost::get<bool>(evaluator_collision_ego.Evaluate(*predicted_world));
        goal_reached = boost::get<bool>(evaluator_goal_reached.Evaluate(*predicted_world));
        out_of_map = false;
         // -------------------------------------------
    } else {
        out_of_map = true;
    }
    // HACK for testing
    // should be solved with https://github.com/bark-simulator/bark/issues/118
    //collision_corridor = false;

    rewards.resize(1);
    rewards[0] = (collision_corridor || collision_ego || out_of_map) * -1000.0f + goal_reached * 1.0f;
   

    bool is_terminal = (collision_corridor || collision_ego || goal_reached || out_of_map);

    return std::make_shared<MctsStateSingleAgent>(predicted_world, is_terminal, num_ego_actions_, prediction_time_span_);
}

mcts::ActionIdx MctsStateSingleAgent::get_num_actions( mcts::AgentIdx agent_idx) const {
    return num_ego_actions_;
}

bool MctsStateSingleAgent::is_terminal() const {
    return is_terminal_state_;
}

const std::vector< mcts::AgentIdx> MctsStateSingleAgent::get_agent_idx() const {
    return std::vector< mcts::AgentIdx>{0};
}

std::string MctsStateSingleAgent::sprintf() const {
    return std::string();
}










}  // namespace behavior
}  // namespace models
}  // namespace modules

