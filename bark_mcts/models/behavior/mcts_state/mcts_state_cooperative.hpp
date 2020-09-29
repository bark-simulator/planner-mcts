// Copyright (c) 2019 Julian Bernhard
// 
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.
// ========================================================

#ifndef BARK_MCTS_COOPERATIVE_STATE_H_
#define BARK_MCTS_COOPERATIVE_STATE_H_

// BARK
#include "bark/models/behavior/behavior_model.hpp"
#include "bark/models/behavior/motion_primitives/motion_primitives.hpp"
// MCTS Library
#include "bark_mcts/models/behavior/mcts_state/mcts_state_base.hpp"

namespace bark {
namespace world {
class ObservedWorld;
typedef std::shared_ptr<ObservedWorld> ObservedWorldPtr;
}  // namespace world
namespace models {
namespace behavior {

using BarkAction = bark::models::behavior::Action;

class MctsStateCooperative : public MctsStateBase<MctsStateCooperative> {
  public:
    MctsStateCooperative(const bark::world::ObservedWorldPtr& observed_world,
                       bool is_terminal_state,
                       const mcts::ActionIdx& num_ego_actions,
                       const float& prediction_time_span,
                       const mcts::AgentIdx& ego_agent_id,
                       const StateParameters& state_parameters);

    std::shared_ptr<MctsStateCooperative> execute(const mcts::JointAction& joint_action,
                                            std::vector<mcts::Reward>& rewards,
                                            mcts::Cost& ego_cost) const;

    std::shared_ptr<MctsStateCooperative> clone() const;

    mcts::ActionIdx get_num_actions(mcts::AgentIdx agent_idx) const { 
        return num_ego_actions_;
    }


 protected:
    ObservedWorldPtr predict(const mcts::JointAction& joint_action) const;

    std::shared_ptr<MctsStateCooperative> generate_next_state(const ObservedWorldPtr& predicted_world,
                                                          std::vector<mcts::Reward>& rewards,  mcts::Cost& ego_cost) const;
};

}  // namespace behavior
}  // namespace models
}  // namespace bark

#endif // BARK_MCTS_COOPERATIVE_STATE_H_