// Copyright (c) 2019 Julian Bernhard
// 
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.
// ========================================================

#ifndef ACTION_VALUE_RANDOM_HEURISTIC_H
#define ACTION_VALUE_RANDOM_HEURISTIC_H

#include "mcts/mcts.h"
#include "mcts/heuristic.h"
#include "bark_ml/observers/base_observer.hpp"
#include "bark_ml/library_wrappers/lib_fqf_iqn_qrdqn/model_loader/model_loader.hpp"
#include <type_traits>
#include <iostream>
#include <chrono>

 namespace mcts {
class CostConstrainedStatistic;

class NNHeuristic :  public mcts::Heuristic<NNHeuristic>
{
public:
    NNHeuristic(const MctsParameters& mcts_parameters) :
            mcts::Heuristic<NNHeuristic>(mcts_parameters), 
            model_loader_(),
             {}

    template<class S, class SE, class SO, class H>
    std::pair<SE, std::unordered_map<AgentIdx, SO>> calculate_heuristic_values(const std::shared_ptr<StageNode<S,SE,SO,H>> &node) {
        namespace chr = std::chrono;
        auto start = std::chrono::high_resolution_clock::now();
        std::shared_ptr<S> state = node->get_state()->clone();


        // generate an extra node statistic for each agent
        SE ego_heuristic(0, node->get_state()->get_ego_agent_idx(), mcts_parameters_);
        if constexpr(std::is_same<SE, CostConstrainedStatistic>::value) {
          ego_heuristic.set_heuristic_estimate(action_returns, action_costs, action_executed_step_lengths);
          //VLOG_EVERY_N(6, 10) << "accum cost = " << action_costs << ", heuristic_step_length = " << action_executed_step_lengths;
          } else {
          ego_heuristic.set_heuristic_estimate(action_returns, action_costs); 
        }
        std::unordered_map<AgentIdx, SO> other_heuristic_estimates;
        AgentIdx reward_idx=1;
        EgoCosts mean_cost(state->get_num_costs(), 0.0);
        for (const auto& action_cost : action_costs) {
          mean_cost += action_cost.second;
        }
        for(auto&  cost : mean_cost) {
          cost /= action_costs.size();
        }
        for (auto agent_idx : node->get_state()->get_other_agent_idx())
        {
            SO statistic(0, agent_idx, mcts_parameters_);
            statistic.set_heuristic_estimate(other_accum_rewards[agent_idx], mean_cost);
            other_heuristic_estimates.insert(std::pair<AgentIdx, SO>(agent_idx, statistic));
            reward_idx++;
        }
        return std::pair<SE, std::unordered_map<AgentIdx, SO>>(ego_heuristic, other_heuristic_estimates);
    }

    private:
        std::unique_ptr<bark_ml::lib_fqf_iqn_qrdqn::ModelLoader> model_loader_;
        std::shared_ptr<bark_ml::observers::BaseObserver> observer_;
};

 } // namespace mcts

#endif