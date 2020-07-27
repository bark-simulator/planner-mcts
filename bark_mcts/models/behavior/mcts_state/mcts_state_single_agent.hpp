// Copyright (c) 2019 fortiss GmbH
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef SINGLE_AGENT_MCTS_STATE_HPP_
#define SINGLE_AGENT_MCTS_STATE_HPP_

#include <iostream>

#include "mcts/state.h"

namespace bark {
namespace world {
class ObservedWorld;
typedef std::shared_ptr<ObservedWorld> ObservedWorldPtr;
}  // namespace world
namespace models {
namespace behavior {
using bark::world::ObservedWorld;
using bark::world::ObservedWorldPtr;

// A simple environment with a 1D state, only if both agents select different
// actions, they get nearer to the terminal state
class MctsStateSingleAgent : public mcts::StateInterface<MctsStateSingleAgent> {
 public:
  MctsStateSingleAgent(const bark::world::ObservedWorldPtr& observed_world,
                       bool is_terminal_state,
                       const mcts::ActionIdx& num_ego_actions,
                       const float& prediction_time_span,
                       const mcts::AgentIdx& ego_agent_id);
  ~MctsStateSingleAgent() {}

  std::shared_ptr<MctsStateSingleAgent> clone() const;

  std::shared_ptr<MctsStateSingleAgent> execute(
      const mcts::JointAction& joint_action, std::vector<mcts::Reward>& rewards,
      mcts::Cost& ego_cost) const;

  mcts::ActionIdx get_num_actions(mcts::AgentIdx agent_idx) const;

  bool is_terminal() const;

  const std::vector<mcts::AgentIdx> get_other_agent_idx() const;

  const mcts::AgentIdx get_ego_agent_idx() const;

  std::string sprintf() const;

  float get_distance_to_goal() const;

  bool get_collision_happen() const;

  const ObservedWorldPtr get_observed_world() const;


 private:
  std::vector<mcts::AgentIdx> update_other_agent_ids() const;

  const std::shared_ptr<bark::world::ObservedWorld> observed_world_;
  const bool is_terminal_state_;
  const mcts::ActionIdx num_ego_actions_;
  const float prediction_time_span_;

  const std::vector<mcts::AgentIdx> other_agent_ids_;
  const mcts::AgentIdx ego_agent_id_;
};

}  // namespace behavior
}  // namespace models
}  // namespace bark

#endif  // SINGLE_AGENT_MCTS_STATE_HPP_
