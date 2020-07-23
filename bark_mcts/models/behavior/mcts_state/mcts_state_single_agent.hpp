// Copyright (c) 2020 Julian Bernhard
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

// A simple environment with a 1D state, only if both agents select different
// actions, they get nearer to the terminal state
class MctsStateSingleAgent : public mcts::StateInterface<MctsStateSingleAgent> {
 public:
  MctsStateSingleAgent(const bark::world::ObservedWorldPtr& observed_world,
                       bool is_terminal_state,
                       const mcts::ActionIdx& num_ego_actions,
                       const float& prediction_time_span);
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

 private:
  const std::shared_ptr<const bark::world::ObservedWorld> observed_world_;
  const bool is_terminal_state_;
  const mcts::ActionIdx num_ego_actions_;
  const float prediction_time_span_;
};

}  // namespace behavior
}  // namespace models
}  // namespace bark

#endif  // SINGLE_AGENT_MCTS_STATE_HPP_
