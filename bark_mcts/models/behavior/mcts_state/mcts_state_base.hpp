// Copyright (c) 2019 Julian Bernhard
// 
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.
// ========================================================

#ifndef BARK_MCTS_STATE_BASE_H_
#define BARK_MCTS_STATE_BASE_H_

// BARK
#include "bark/world/observed_world.hpp"
#include "bark/models/behavior/behavior_model.hpp"
#include "bark/models/behavior/motion_primitives/motion_primitives.hpp"

// MCTS Library
#include "mcts/hypothesis/hypothesis_state.h"

namespace bark {
namespace world {
class ObservedWorld;
typedef std::shared_ptr<ObservedWorld> ObservedWorldPtr;
}  // namespace world
namespace models {
namespace behavior {

using BarkAction = bark::models::behavior::Action;
using bark::world::ObservedWorld;
using bark::world::ObservedWorldPtr;

typedef struct StateParameters {
  float GOAL_REWARD;
  float COLLISION_REWARD;
  float GOAL_COST;
  float COLLISION_COST;
} StateParameters;

typedef struct EvaluationResults {
  EvaluationResults() : 
      collision_other_agent(false),
      collision_drivable_area(false),
      goal_reached(false),
      out_of_map(false),
      is_terminal(false) {}
  bool collision_other_agent;
  bool collision_drivable_area;
  bool goal_reached;
  bool out_of_map;
  bool is_terminal;
} EvaluationResults;

template<class T>
class MctsStateBase : public mcts::HypothesisStateInterface<T> {
  public:
    MctsStateBase(const bark::world::ObservedWorldPtr& observed_world,
                       bool is_terminal_state,
                       const mcts::ActionIdx& num_ego_actions,
                       const float& prediction_time_span,
                       const BehaviorMotionPrimitivesPtr& ego_behavior_model,
                       const mcts::AgentIdx& ego_agent_id,
                       const StateParameters& state_parameters,
                       const std::unordered_map<mcts::AgentIdx, mcts::HypothesisId>& current_agents_hypothesis);

    mcts::ActionIdx get_num_actions(mcts::AgentIdx agent_idx) const { return num_ego_actions_; }

    bool is_terminal() const { return is_terminal_state_; };

    const std::vector<mcts::AgentIdx> get_other_agent_idx() const;

    const mcts::AgentIdx get_ego_agent_idx() const;

    std::string sprintf() const;

    const ObservedWorld& get_observed_world() const { return *observed_world_; }

 protected:
  std::vector<mcts::AgentIdx> update_other_agent_ids() const;

  ObservedWorldPtr predict(const mcts::JointAction& joint_action) const;

  EvaluationResults evaluate(const ObservedWorld& observed_world) const;

  std::shared_ptr<T> generate_next_state(const EvaluationResults& evaluation_results, const ObservedWorldPtr& predicted_world,
                                                        std::vector<mcts::Reward>& rewards,  mcts::Cost& ego_cost) const;

  const std::shared_ptr<const bark::world::ObservedWorld> observed_world_;
  const bool is_terminal_state_;
  const mcts::ActionIdx num_ego_actions_;
  const float prediction_time_span_;
  const std::vector<mcts::AgentIdx> other_agent_ids_;
  const mcts::AgentIdx ego_agent_id_;
  const StateParameters state_parameters_;
  const BehaviorMotionPrimitivesPtr& ego_behavior_model_;
};

template<class T>
MctsStateBase<T>::MctsStateBase(const bark::world::ObservedWorldPtr& observed_world,
                       bool is_terminal_state,
                       const mcts::ActionIdx& num_ego_actions,
                       const float& prediction_time_span,
                       const BehaviorMotionPrimitivesPtr& ego_behavior_model,
                       const mcts::AgentIdx& ego_agent_id,
                       const StateParameters& state_parameters,
                       const std::unordered_map<mcts::AgentIdx, mcts::HypothesisId>& current_agents_hypothesis) :
      mcts::HypothesisStateInterface<T>(current_agents_hypothesis),
      observed_world_(observed_world),
      is_terminal_state_(is_terminal_state),
      num_ego_actions_(num_ego_actions),
      prediction_time_span_(prediction_time_span),
      ego_behavior_model_(ego_behavior_model),
      other_agent_ids_(update_other_agent_ids()),
      ego_agent_id_(ego_agent_id),
      state_parameters_(state_parameters) {}

template<class T>
const std::vector<mcts::AgentIdx> MctsStateBase<T>::get_other_agent_idx() const {
  return other_agent_ids_;
}

template<class T>
const mcts::AgentIdx MctsStateBase<T>::get_ego_agent_idx() const {
  return ego_agent_id_;
}

template<class T>
std::vector<mcts::AgentIdx> MctsStateBase<T>::update_other_agent_ids() const {
  world::AgentMap agent_map = observed_world_->GetOtherAgents();
  std::vector<mcts::AgentIdx> ids;
  for (const auto &agent : agent_map) {
    ids.push_back(agent.first);
  }
  return ids;
}

template<class T>
std::string MctsStateBase<T>::sprintf() const {
    std::stringstream ss;
    for ( const auto& agent : observed_world_->GetAgents()) {
         ss << "Agent " << agent.first << ", State: " << agent.second->GetCurrentState() << 
          ", Action: " <<  boost::apply_visitor(action_tostring_visitor(), agent.second->GetBehaviorModel()->GetLastAction()) ;
    }
    return ss.str();
}

template<class T>
ObservedWorldPtr MctsStateBase<T>::predict(const mcts::JointAction& joint_action) const {
  return mcts::StateInterface<T>::impl().predict(joint_action);
}

template<class T>
EvaluationResults MctsStateBase<T>::evaluate(const ObservedWorld& observed_world) const {
  return mcts::StateInterface<T>::impl().evaluate(observed_world);
}

template<class T>
std::shared_ptr<T> MctsStateBase<T>::generate_next_state(const EvaluationResults& evaluation_results, const ObservedWorldPtr& predicted_world,
                                                         std::vector<mcts::Reward>& rewards,  mcts::Cost& ego_cost) const {
  return mcts::StateInterface<T>::impl().generate_next_state(evaluation_results, predicted_world, rewards, ego_cost);
}


}  // namespace behavior
}  // namespace models
}  // namespace bark

#endif // BARK_MCTS_HYPOTHESIS_STATE_H_