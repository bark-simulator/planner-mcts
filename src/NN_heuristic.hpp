// Copyright (c) 2019 Julian Bernhard
// 
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.
// ========================================================

#ifndef NEURALNETWORK_BASED_HEURISTIC_HPP
#define NEURALNETWORK_BASED_HEURISTIC_HPP

#include "mcts/mcts.h"
#include <iostream>
#include <chrono>
#include <math.h>
#include <cmath>
#include "src/observers/nearest_observer_new.hpp"

namespace modules {
namespace models {
namespace behavior {
// assumes all agents have equal number of actions and the same node statistic
class NNHeuristic :  public mcts::Heuristic<NNHeuristic>
{
public:
    NNHeuristic(const mcts::MctsParameters& mcts_parameters) :
            mcts::Heuristic<NNHeuristic>(mcts_parameters) {
                NearestObserver Observer1();
            }
    static void InitializeModelLoader(std::string model_directory)

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
        
        modules::world::ObservedWorldPtr observed_world = node->get_state()->get_observed_world(); //<- dp as we did with get nearest distance
        ObservedState output = Observer1.observe(observed_world); //call observe method        
        std::vector<float> model_output = model_loader_ptr->Evaluator(output);
        
        double num_actions = model_output.size(); //num actions //use model.output size
        double value = std::accumulate(model_output.begin(), model_output.end(), 0);
            
            
            

        mcts::Reward ego_all_reward = 5*(1/num_actions)*value;


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
    static void InitializeModelLoader();
        void InitializeModelLoader() {
            model_loader_ptr = new ModelLoader();
            //model_loader_ptr->LoadModel();
        }

    private:

    static ModelLoader* model_loader_ptr;
    NearestObserver Observer1;


};


}  // namespace behavior
}  // namespace models
}  // namespace modules

#endif // NEURALNETWORK_BASED_HEURISTIC_HPP