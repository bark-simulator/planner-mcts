// Copyright (c) 2019 Julian Bernhard
// 
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.
// ========================================================

#ifndef BARK_MCTS_STATE_BASE_H_
#define BARK_MCTS_STATE_BASE_H_

// BARK
#include "bark/world/evaluation/evaluator_collision_ego_agent.hpp"
#include "bark/world/evaluation/evaluator_drivable_area.hpp"
#include "bark/world/evaluation/evaluator_goal_reached.hpp"
#include "bark/world/evaluation/safe_distances/evaluator_dynamic_safe_dist_long.hpp"
#include "bark/world/evaluation/safe_distances/evaluator_static_safe_dist.hpp"
#include "bark/world/evaluation/safe_distances/evaluator_safe_dist_drivable_area.hpp"
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
using bark::world::evaluation::EvaluatorCollisionEgoAgent;
using bark::world::evaluation::EvaluatorDrivableArea;
using bark::world::evaluation::EvaluatorGoalReached;
using bark::world::evaluation::EvaluatorDynamicSafeDistLong;
using bark::world::evaluation::EvaluatorStaticSafeDist;
using bark::world::evaluation::EvaluatorSafeDistDrivableArea;


typedef struct EvaluationParameters {
  EvaluationParameters() : add_safe_dist(false), static_safe_dist_is_terminal(false),
        dynamic_safe_dist_is_terminal(false), evaluator_dynamic_safe_dist_long(nullptr), 
        evaluator_static_safe_dist(nullptr),
        evaluator_safe_dist_drivable_area(nullptr)  {}
  EvaluationParameters(bool add_safe_dist, bool static_safe_dist_is_terminal,
               bool dynamic_safe_dist_is_terminal, const bark::commons::ParamsPtr& params) : 
               add_safe_dist(add_safe_dist),
               static_safe_dist_is_terminal(static_safe_dist_is_terminal),
               dynamic_safe_dist_is_terminal(dynamic_safe_dist_is_terminal),
         evaluator_dynamic_safe_dist_long(std::make_shared<EvaluatorDynamicSafeDistLong>(params, std::numeric_limits<AgentId>::max())),
         evaluator_static_safe_dist(std::make_shared<EvaluatorStaticSafeDist>(params, std::numeric_limits<AgentId>::max())),
         evaluator_safe_dist_drivable_area(std::make_shared<EvaluatorSafeDistDrivableArea>(params, std::numeric_limits<AgentId>::max()))  {}
  bool add_safe_dist;
  bool static_safe_dist_is_terminal;
  bool dynamic_safe_dist_is_terminal;
  std::shared_ptr<EvaluatorDynamicSafeDistLong> evaluator_dynamic_safe_dist_long;
  std::shared_ptr<EvaluatorStaticSafeDist> evaluator_static_safe_dist;
  std::shared_ptr<EvaluatorSafeDistDrivableArea> evaluator_safe_dist_drivable_area;
} EvaluationParameters;

typedef struct StateParameters {
  float GOAL_REWARD;
  float COLLISION_REWARD;
  float SAFE_DIST_VIOLATED_REWARD;
  float GOAL_COST;
  float COLLISION_COST;
  float SAFE_DIST_VIOLATED_COST;
  float COOPERATION_FACTOR;
  float STEP_REWARD;
  bool split_safe_dist_collision;
  bool chance_costs;
  EvaluationParameters evaluation_parameters;
} StateParameters;



typedef struct EvaluationResults {
  EvaluationResults() : 
      collision_other_agent(false),
      collision_drivable_area(false),
      static_safe_distance_violated(false),
      dynamic_safe_distance_violated(false),
      goal_reached(false),
      out_of_map(false),
      is_terminal(false) {}
  bool collision_other_agent;
  bool static_safe_distance_violated;
  bool dynamic_safe_distance_violated;
  bool collision_drivable_area;
  bool goal_reached;
  bool out_of_map;
  bool is_terminal;
} EvaluationResults;

inline mcts::Reward reward_from_evaluation_results(const EvaluationResults& evaluation_results, const StateParameters& parameters) {
  const mcts::Reward safe_dist_reward = float(evaluation_results.dynamic_safe_distance_violated || evaluation_results.static_safe_distance_violated) 
                                    * parameters.SAFE_DIST_VIOLATED_REWARD;
  const bool use_collision = !(parameters.evaluation_parameters.static_safe_dist_is_terminal &&
                     parameters.evaluation_parameters.dynamic_safe_dist_is_terminal);
  const mcts::Reward collision_reward = float( (use_collision ? evaluation_results.collision_drivable_area : false) || evaluation_results.collision_other_agent || 
                evaluation_results.out_of_map ) * parameters.COLLISION_REWARD;
  return collision_reward + (parameters.evaluation_parameters.add_safe_dist ? safe_dist_reward : 0.0f) +
          float(evaluation_results.goal_reached) * parameters.GOAL_REWARD + parameters.STEP_REWARD;
};

inline mcts::EgoCosts ego_costs_from_evaluation_results(const EvaluationResults& evaluation_results, const StateParameters& parameters) {
  mcts::EgoCosts ego_costs(2, 0.0f);
  const mcts::Cost safe_dist_cost = float(parameters.evaluation_parameters.static_safe_dist_is_terminal ? 
                              evaluation_results.dynamic_safe_distance_violated :
                               evaluation_results.dynamic_safe_distance_violated || evaluation_results.static_safe_distance_violated) *  parameters.SAFE_DIST_VIOLATED_COST;
  const mcts::Cost total_costs = float(evaluation_results.collision_drivable_area || evaluation_results.collision_other_agent || evaluation_results.out_of_map ||
         (parameters.evaluation_parameters.static_safe_dist_is_terminal ?  evaluation_results.static_safe_distance_violated : false)||
        (parameters.evaluation_parameters.dynamic_safe_dist_is_terminal ?  evaluation_results.dynamic_safe_distance_violated : false)) * parameters.COLLISION_COST +
        float((parameters.evaluation_parameters.add_safe_dist && !parameters.split_safe_dist_collision) ? safe_dist_cost : 0.0f) +
          evaluation_results.goal_reached * parameters.GOAL_COST;
  if(parameters.split_safe_dist_collision) {
    ego_costs[0] = parameters.chance_costs ? std::min(safe_dist_cost, 1.0) : safe_dist_cost; // The constrained policy is always calculated over the first index
    ego_costs[1] = parameters.chance_costs ? std::min(total_costs, 1.0) : total_costs;
  } else {
    ego_costs[0] = parameters.chance_costs ? std::min(total_costs, 1.0) : total_costs; // The constrained policy is always calculated over the first index
    ego_costs[1] = 0.0f;
  }
  return ego_costs;
};

template<class T>
class MctsStateBase : public mcts::HypothesisStateInterface<T> {
  public:
    MctsStateBase(const bark::world::ObservedWorldPtr& observed_world,
                       bool is_terminal_state,
                       const mcts::ActionIdx& num_ego_actions,
                       const float& prediction_time_span,
                       const mcts::AgentIdx& ego_agent_id,
                       const StateParameters& state_parameters,
                       const std::unordered_map<mcts::AgentIdx, mcts::HypothesisId>& current_agents_hypothesis);

    bool is_terminal() const { return is_terminal_state_; };

    const std::vector<mcts::AgentIdx> get_other_agent_idx() const;

    const mcts::AgentIdx get_ego_agent_idx() const;

    const std::size_t get_num_costs() const;

    std::string sprintf() const;

    const ObservedWorld& get_observed_world() const { return *observed_world_; }

 protected:
  std::vector<mcts::AgentIdx> update_other_agent_ids() const;

  const std::shared_ptr<const bark::world::ObservedWorld> observed_world_;
  const bool is_terminal_state_;
  const mcts::ActionIdx num_ego_actions_;
  const float prediction_time_span_;
  const std::vector<mcts::AgentIdx> other_agent_ids_;
  const mcts::AgentIdx ego_agent_id_;
  const StateParameters state_parameters_;
};

template<class T>
MctsStateBase<T>::MctsStateBase(const bark::world::ObservedWorldPtr& observed_world,
                       bool is_terminal_state,
                       const mcts::ActionIdx& num_ego_actions,
                       const float& prediction_time_span,
                       const mcts::AgentIdx& ego_agent_id,
                       const StateParameters& state_parameters,
                       const std::unordered_map<mcts::AgentIdx, mcts::HypothesisId>& current_agents_hypothesis) :
      mcts::HypothesisStateInterface<T>(current_agents_hypothesis),
      observed_world_(observed_world),
      is_terminal_state_(is_terminal_state),
      num_ego_actions_(num_ego_actions),
      prediction_time_span_(prediction_time_span),
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
const std::size_t MctsStateBase<T>::get_num_costs() const {
  return 2;
}

template<class T>
std::vector<mcts::AgentIdx> MctsStateBase<T>::update_other_agent_ids() const {
  world::AgentMap agent_map = observed_world_->GetValidOtherAgents();
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

inline EvaluationResults mcts_observed_world_evaluation(const ObservedWorld& observed_world, const EvaluationParameters& evaluation_parameters) {
  EvaluationResults evaluation_results;

  if (observed_world.GetEgoAgent()) {
    auto ego_id = observed_world.GetEgoAgent()->GetAgentId();
    const auto evaluator_drivable_area = evaluation_parameters.evaluator_safe_dist_drivable_area;
    auto evaluator_collision_ego = EvaluatorCollisionEgoAgent(ego_id);
    auto evaluator_goal_reached = EvaluatorGoalReached(ego_id);

    evaluation_results.collision_drivable_area =
        !boost::get<bool>(evaluator_drivable_area->CheckSafeDistance(observed_world));
    evaluation_results.collision_other_agent =
        boost::get<bool>(evaluator_collision_ego.Evaluate(observed_world));
    evaluation_results.goal_reached =
        boost::get<bool>(evaluator_goal_reached.Evaluate(observed_world));
    evaluation_results.out_of_map = false;

    if(evaluation_parameters.add_safe_dist) {
      evaluation_results.dynamic_safe_distance_violated = !boost::get<bool>(evaluation_parameters.evaluator_dynamic_safe_dist_long->CheckSafeDistance(observed_world));
      evaluation_results.static_safe_distance_violated = !boost::get<bool>(evaluation_parameters.evaluator_static_safe_dist->CheckSafeDistance(observed_world));
    }

  } else {
    evaluation_results.out_of_map = true;
  }
  evaluation_results.is_terminal = (evaluation_results.collision_drivable_area || evaluation_results.collision_other_agent
                                   || evaluation_results.goal_reached || evaluation_results.out_of_map) || 
                            (evaluation_parameters.static_safe_dist_is_terminal ? evaluation_results.static_safe_distance_violated : false) || 
                            (evaluation_parameters.dynamic_safe_dist_is_terminal ? evaluation_results.dynamic_safe_distance_violated : false);
  return evaluation_results;
}

}  // namespace behavior
}  // namespace models
}  // namespace bark

#endif // BARK_MCTS_HYPOTHESIS_STATE_H_