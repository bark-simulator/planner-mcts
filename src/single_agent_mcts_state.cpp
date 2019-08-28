// Copyright (c) 2019 fortiss GmbH
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "src/single_agent_mcts_state.hpp"
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

SingleAgentMCTSState::SingleAgentMCTSState(const ObservedWorldPtr& observed_world, bool is_terminal_state,
                                           const mcts::ActionIdx& num_ego_actions,
                                           const float& prediction_time_span) : 
                                        observed_world_(observed_world),
                                        is_terminal_state_(is_terminal_state),
                                        num_ego_actions_(num_ego_actions),
                                        prediction_time_span_(prediction_time_span) {}

std::shared_ptr<SingleAgentMCTSState> SingleAgentMCTSState::clone() const {
    return std::make_shared<SingleAgentMCTSState>(ObservedWorldPtr(observed_world_->Clone()),
                             is_terminal_state_, num_ego_actions_, prediction_time_span_);
}

std::shared_ptr<SingleAgentMCTSState> SingleAgentMCTSState::execute(const mcts::JointAction& joint_action,
                                                                    std::vector< mcts::Reward>& rewards ) const {

    // TODO: parameter for prediction time span
    auto predicted_world = observed_world_->Predict(prediction_time_span_,
                                 DiscreteAction(joint_action[SingleAgentMCTSState::ego_agent_idx]));

    // TODO: allow for separate configuration options ------
    auto evaluator_collision_corridor = EvaluatorCollisionDrivingCorridor();
    auto evaluator_collision_ego = EvaluatorCollisionEgoAgent(predicted_world->get_ego_agent()->get_agent_id());
    auto evaluator_goal_reached = EvaluatorGoalReached(predicted_world->get_ego_agent()->get_agent_id());
    
    bool collision_corridor = boost::get<bool>(evaluator_collision_corridor.Evaluate(*predicted_world));
    bool collision_ego = boost::get<bool>(evaluator_collision_ego.Evaluate(*predicted_world));
    bool goal_reached = boost::get<bool>(evaluator_goal_reached.Evaluate(*predicted_world));

    rewards.resize(1);
    rewards[0] = (collision_corridor || collision_ego) * -1000.0f + goal_reached * 1.0f;
    // -------------------------------------------

    bool is_terminal = (collision_corridor || collision_ego || goal_reached);

    return std::make_shared<SingleAgentMCTSState>(predicted_world, is_terminal, num_ego_actions_, prediction_time_span_);
}

mcts::ActionIdx SingleAgentMCTSState::get_num_actions( mcts::AgentIdx agent_idx) const {
    return num_ego_actions_;
}

bool SingleAgentMCTSState::is_terminal() const {
    return is_terminal_state_;
}

const std::vector< mcts::AgentIdx> SingleAgentMCTSState::get_agent_idx() const {
    return std::vector< mcts::AgentIdx>{0};
}

std::string SingleAgentMCTSState::sprintf() const {
    return std::string();
}










}  // namespace behavior
}  // namespace models
}  // namespace modules

