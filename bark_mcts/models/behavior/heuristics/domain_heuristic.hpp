// Copyright (c) 2019 Julian Bernhard
// 
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.
// ========================================================

#ifndef DOMAIN_BASED_HEURISTIC_HPP
#define DOMAIN_BASED_HEURISTIC_HPP

#include "mcts/mcts.h"
#include <iostream>
#include <chrono>
#include <math.h>
#include <cmath>

namespace modules {
namespace models {
namespace behavior {
// assumes all agents have equal number of actions and the same node statistic
class DomainHeuristic :  public mcts::Heuristic<DomainHeuristic>
{
public:
    DomainHeuristic(const mcts::MctsParameters& mcts_parameters) :
            mcts::Heuristic<DomainHeuristic>(mcts_parameters) {}

    template<class S, class SE, class SO, class H>
    std::pair<SE, std::unordered_map<mcts::AgentIdx, SO>> calculate_heuristic_values(const std::shared_ptr<mcts::StageNode<S,SE,SO,H>> &node) {
        //catch case where newly expanded state is terminal
        if(node->get_state()->is_terminal()){
            const auto ego_agent_idx = node->get_state()->get_ego_agent_idx();
            const mcts::ActionIdx num_ego_actions = node->get_state()->get_num_actions(ego_agent_idx); 
            SE ego_heuristic(num_ego_actions, node->get_state()->get_ego_agent_idx(), mcts_parameters_);
            ego_heuristic.set_heuristic_estimate(100.0f, 0.0f);//(0.0f, 0.0f)
            std::unordered_map<mcts::AgentIdx, SO> other_heuristic_estimates;
            for (const auto& ai : node->get_state()->get_other_agent_idx())
            { 
              SO statistic(node->get_state()->get_num_actions(ai), ai, mcts_parameters_);
              statistic.set_heuristic_estimate(0.0f, 0.0f);
              other_heuristic_estimates.insert(std::pair<mcts::AgentIdx, SO>(ai, statistic));
            }
            return std::pair<SE, std::unordered_map<mcts::AgentIdx, SO>>(ego_heuristic, other_heuristic_estimates) ;
        }
    
        // generate an extra node statistic for each agent
        SE ego_heuristic(0, node->get_state()->get_ego_agent_idx(), mcts_parameters_);
        auto goal_distance = node->get_state()->get_distance_to_goal();
        mcts::Reward ego_all_reward = 105-5*exp(0.3*goal_distance);
        //mcts::Reward ego_all_reward = 1/(0.01 + goal_distance);
        //mcts::Reward ego_all_reward = 100-80*goal_distance;
        //mcts::Reward ego_all_reward = 100-100*log(goal_distance+1);


        ego_heuristic.set_heuristic_estimate(ego_all_reward, -ego_all_reward);//(ego_all_reward, -ego_all_reward)
        LOG_EVERY_N(INFO, 100) << "Calculating domain value=" << ego_all_reward << ", for dist. to. goal=" << goal_distance;//30
        std::unordered_map<mcts::AgentIdx, SO> other_heuristic_estimates;
        mcts::AgentIdx reward_idx=1;
        for (auto agent_idx : node->get_state()->get_other_agent_idx())
        {
            SO statistic(0, agent_idx, mcts_parameters_);
            statistic.set_heuristic_estimate(0.0f, 0.0f);
            other_heuristic_estimates.insert(std::pair<mcts::AgentIdx, SO>(agent_idx, statistic));
            reward_idx++;
        }
        return std::pair<SE, std::unordered_map<mcts::AgentIdx, SO>>(ego_heuristic, other_heuristic_estimates);
    }  

};


}  // namespace behavior
}  // namespace models
}  // namespace modules

#endif // DOMAIN_BASED_HEURISTIC_HPP