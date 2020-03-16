// Copyright (c) 2019 fortiss GmbH
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "src/mcts_state_hypothesis.hpp"

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

MctsStateHypothesis::MctsStateHypothesis(
                       const modules::world::ObservedWorldPtr& observed_world,
                       bool is_terminal_state,
                       const mcts::ActionIdx& num_ego_actions,
                       const float& prediction_time_span,
                       const std::unordered_map<mcts::AgentIdx, mcts::HypothesisId>& current_agents_hypothesis,
                       const std::vector<BehaviorHypothesisPtr>& behavior_hypothesis,
                       const BehaviorMotionPrimitivesPtr& ego_behavior_model) :
      mcts::HypothesisStateInterface<MctsStateHypothesis>(current_agents_hypothesis),
      observed_world_(observed_world),
      is_terminal_state_(is_terminal_state),
      num_ego_actions_(num_ego_actions),
      prediction_time_span_(prediction_time_span),
      behavior_hypothesis_(behavior_hypothesis),
      ego_behavior_model_(ego_behavior_model) {
          auto agent_ids = get_agent_idx();
          // start at index 1 since first agent is ego agent
          for(size_t ai=1; ai < agent_ids.size(); ++ai) {
              behaviors_stored_[ai] = std::make_shared<BehaviorActionStore>(nullptr);
          }
      }

std::shared_ptr<MctsStateHypothesis> MctsStateHypothesis::clone() const {
  auto worldptr =
      std::dynamic_pointer_cast<ObservedWorld>(observed_world_->Clone());
  return std::make_shared<MctsStateHypothesis>(
      worldptr, is_terminal_state_, num_ego_actions_, prediction_time_span_,
      current_agents_hypothesis_, behavior_hypothesis_, ego_behavior_model_);
}

std::shared_ptr<MctsStateHypothesis> MctsStateHypothesis::execute(
    const mcts::JointAction& joint_action, std::vector<mcts::Reward>& rewards,
    mcts::Cost& ego_cost) const {
  BARK_EXPECT_TRUE(!is_terminal());

  // pass next actions to behavior models for prediction
  ego_behavior_model_->ActionToBehavior(DiscreteAction(joint_action[this->ego_agent_idx]));
  auto agent_ids = get_agent_idx();
  auto predicted_behaviors_ = behaviors_stored_;
  for (size_t i = 1; i < agent_ids.size(); ++i) {
      std::dynamic_pointer_cast<BehaviorActionStore>(predicted_behaviors_[agent_ids[i]])
            ->MakeBehaviorActive(static_cast<ActionHash>(joint_action[i]));
  }
  const auto predicted_world = 
            observed_world_->Predict(prediction_time_span_, ego_behavior_model_, predicted_behaviors_);

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

  return std::make_shared<MctsStateHypothesis>(
      predicted_world, is_terminal, num_ego_actions_, prediction_time_span_,
      current_agents_hypothesis_, behavior_hypothesis_, ego_behavior_model_);
}

const std::vector<mcts::AgentIdx> MctsStateHypothesis::get_agent_idx() const {
  return agent_ids_;
}


mcts::ActionIdx MctsStateHypothesis::plan_action_current_hypothesis(const mcts::AgentIdx& agent_idx) const {
    const mcts::HypothesisId agt_hyp_id = this->current_agents_hypothesis_.at(agent_idx);
    const auto& trajectory = behavior_hypothesis_[agt_hyp_id]->Plan(prediction_time_span_, *observed_world_);
    const BarkAction bark_action = behavior_hypothesis_[agt_hyp_id]->GetLastAction();
    auto agent_ids = get_agent_idx();
    const mcts::ActionIdx mcts_action = std::dynamic_pointer_cast<BehaviorActionStore>(
        behaviors_stored_[agent_ids_[agent_idx]])->Store(bark_action, trajectory);
    return mcts_action;
}

mcts::Probability get_probability(const mcts::HypothesisId& hypothesis, const mcts::AgentIdx& agent_idx, 
            const modules::models::behavior::Action& action) const {
    if (agent_idx == this->ego_agent_idx) {
        return 0.0f;
    } else {
        return static_cast<mcts::Probability>(
            behavior_hypothesis_[hypothesis]->GetProbability(action, *observed_world_, agent_idx));
    }
}

mcts::Probability MctsStateHypothesis::get_prior(const mcts::HypothesisId& hypothesis, const mcts::AgentIdx& agent_idx) const { return 0.5f;};

mcts::HypothesisId MctsStateHypothesis::get_num_hypothesis(const mcts::AgentIdx& agent_idx) const {return behavior_hypothesis_.size();};



}  // namespace behavior
}  // namespace models
}  // namespace modules
