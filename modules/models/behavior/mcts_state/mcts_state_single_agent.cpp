// Copyright (c) 2019 fortiss GmbH
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "src/mcts_state_single_agent.hpp"

#include <memory>
#include <string>
#include <vector>
#include "modules/world/evaluation/evaluator_collision_ego_agent.hpp"
#include "modules/world/evaluation/evaluator_drivable_area.hpp"
#include "modules/world/evaluation/evaluator_goal_reached.hpp"
#include "modules/world/observed_world.hpp"

namespace modules {
namespace models {
namespace behavior {

using modules::world::ObservedWorld;
using modules::world::ObservedWorldPtr;
using modules::world::evaluation::EvaluatorCollisionEgoAgent;
using modules::world::evaluation::EvaluatorDrivableArea;
using modules::world::evaluation::EvaluatorGoalReached;

MctsStateSingleAgent::MctsStateSingleAgent(
    const ObservedWorldPtr& observed_world, bool is_terminal_state,
    const mcts::ActionIdx& num_ego_actions, const float& prediction_time_span)
    : observed_world_(observed_world),
      is_terminal_state_(is_terminal_state),
      num_ego_actions_(num_ego_actions),
      prediction_time_span_(prediction_time_span) {}

std::shared_ptr<MctsStateSingleAgent> MctsStateSingleAgent::clone() const {
  auto worldptr =
      std::dynamic_pointer_cast<ObservedWorld>(observed_world_->Clone());
  return std::make_shared<MctsStateSingleAgent>(
      worldptr, is_terminal_state_, num_ego_actions_, prediction_time_span_);
}

std::shared_ptr<MctsStateSingleAgent> MctsStateSingleAgent::execute(
    const mcts::JointAction& joint_action, std::vector<mcts::Reward>& rewards,
    mcts::Cost& ego_cost) const {
  BARK_EXPECT_TRUE(!is_terminal());

  // TODO: parameter for prediction time span
  const auto predicted_world = observed_world_->Predict(
      prediction_time_span_,
      DiscreteAction(joint_action[MctsStateSingleAgent::ego_agent_idx]));

  bool collision_drivable_area = false;
  bool collision_ego = false;
  bool goal_reached = false;
  bool out_of_map = false;

  if (predicted_world->GetEgoAgent()) {
    // TODO: allow for separate configuration options ------
    auto ego_id = predicted_world->GetEgoAgent()->GetAgentId();
    auto evaluator_drivable_area = EvaluatorDrivableArea();
    auto evaluator_collision_ego = EvaluatorCollisionEgoAgent(ego_id);
    auto evaluator_goal_reached = EvaluatorGoalReached(ego_id);

    collision_drivable_area =
        boost::get<bool>(evaluator_drivable_area.Evaluate(*predicted_world));
    collision_ego =
        boost::get<bool>(evaluator_collision_ego.Evaluate(*predicted_world));
    goal_reached =
        boost::get<bool>(evaluator_goal_reached.Evaluate(*predicted_world));
    out_of_map = false;
    // -------------------------------------------
  } else {
    out_of_map = true;
  }

  rewards.resize(1);
  rewards[0] =
      (collision_drivable_area || collision_ego || out_of_map) * -1000.0f +
      goal_reached * 100.0f;

  bool is_terminal =
      (collision_drivable_area || collision_ego || goal_reached || out_of_map);

  return std::make_shared<MctsStateSingleAgent>(
      predicted_world, is_terminal, num_ego_actions_, prediction_time_span_);
}

mcts::ActionIdx MctsStateSingleAgent::get_num_actions(
    mcts::AgentIdx agent_idx) const {
  return num_ego_actions_;
}

bool MctsStateSingleAgent::is_terminal() const { return is_terminal_state_; }

const std::vector<mcts::AgentIdx> MctsStateSingleAgent::get_other_agent_idx() const {
  return std::vector<mcts::AgentIdx>{0};
}

const mcts::AgentIdx MctsStateSingleAgent::get_ego_agent_idx() const {
  return 0;
}

std::string MctsStateSingleAgent::sprintf() const { return std::string(); }

}  // namespace behavior
}  // namespace models
}  // namespace modules
