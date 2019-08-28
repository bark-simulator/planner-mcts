// Copyright (c) 2019 fortiss GmbH
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef SINGLE_AGENT_MCTS_STATE_HPP_
#define SINGLE_AGENT_MCTS_STATE_HPP_

#include <iostream>

#include "modules/world/observed_world.hpp"
#include "mcts/state.h"


namespace modules {
namespace models {
namespace behavior {


// A simple environment with a 1D state, only if both agents select different actions, they get nearer to the terminal state
class SingleAgentMCTSState : public mcts::StateInterface<SingleAgentMCTSState> {
  public:
    SingleAgentMCTSState(const modules::world::ObservedWorldPtr& observed_world, bool is_terminal_state,
                                           const mcts::ActionIdx& num_ego_actions,
                                           const float& prediction_time_span);
    ~SingleAgentMCTSState() {}

    std::shared_ptr<SingleAgentMCTSState> clone() const;

    std::shared_ptr<SingleAgentMCTSState> execute(const mcts::JointAction& joint_action, std::vector< mcts::Reward>& rewards ) const;

    mcts::ActionIdx get_num_actions( mcts::AgentIdx agent_idx) const;

    bool is_terminal() const;

    const std::vector<mcts::AgentIdx> get_agent_idx() const;

    std::string sprintf() const;
  private:
    const std::shared_ptr<const modules::world::ObservedWorld> observed_world_;
    const bool is_terminal_state_;
    const mcts::ActionIdx num_ego_actions_;
    const float prediction_time_span_;
};

}  // namespace behavior
}  // namespace models
}  // namespace modules


#endif // SINGLE_AGENT_MCTS_STATE_HPP_
