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
#include "src/model_loader/ModelLoader.hpp"
#include "bark/commons/params/params.hpp"
#include "bark/geometry/geometry.hpp"
#include "bark/commons/params/setter_params.hpp"

using observers::NearestObserver;
using observers::ObservedState;
using ObservedState = Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>;
using namespace bark::commons;

namespace bark {
namespace models {
namespace behavior {
                  


// assumes all agents have equal number of actions and the same node statistic
class NNHeuristic :  public mcts::Heuristic<NNHeuristic>
{
public:
    NNHeuristic(const mcts::MctsParameters& mcts_parameters) :
            mcts::Heuristic<NNHeuristic>(mcts_parameters) {
            }
    

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
        
        const std::shared_ptr<bark::world::ObservedWorld> observed_world = node->get_state()->get_observed_world(); //<- dp as we did with get nearest distance
        ObservedState output = Observer_ptr->observe(observed_world); //call observe method 
        std::vector<float> output_vector(output.cols());
        
        for (int i=0; i< output.cols(); i++){
            output_vector[i] = output(0, i);
            LOG(INFO) << "observedstate" << i << "=" <<output_vector[i];
            }   
        
        std::vector<float> model_output = model_loader_ptr->Evaluator(output_vector,8);
        
        float num_actions = model_output.size(); //num actions //use model.output size
        float value = std::accumulate(model_output.begin(), model_output.end(), 0.0);
        float scalar = 500.0;
        LOG(INFO) << "value=" << value;
        LOG(INFO) << "1/number of actions=" << (1/num_actions);
        LOG(INFO) << "number of actions=" << num_actions;
        mcts::Reward ego_all_reward = scalar*(1/num_actions)*value*scalar;
        for (int i=0; i< num_actions; i++){
           LOG(INFO) << "q_value for action" << i << "=" <<model_output[i];
            }


        LOG(INFO) << "without scalar Calculating nn_heuristic value before=" << ego_all_reward;//30
        ego_heuristic.set_heuristic_estimate(ego_all_reward, -ego_all_reward);//(ego_all_reward, -ego_all_reward)
        LOG(INFO) << "with scalar Calculating nn_heuristic value=" << ego_all_reward;//30
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
    static void InitializeModelLoader(std::string& model_dir) {
            const char* model_dir_0 = &model_dir[0];
            model_loader_ptr = new ModelLoader(model_dir_0);
            //model_loader_ptr->LoadModel();
        }
    static void InitializeObserver() {
            auto params = std::make_shared<SetterParams>(false);   
            int nearest_agent_num_ = 4;
            params->SetInt("ML::Observer::n_nearest_agents", nearest_agent_num_);
            Observer_ptr = new NearestObserver(params);
   }


private:

    static ModelLoader* model_loader_ptr;
    static NearestObserver* Observer_ptr;
};

ModelLoader* NNHeuristic::model_loader_ptr = NULL;
NearestObserver* NNHeuristic::Observer_ptr = NULL;


}  // namespace behavior
}  // namespace models
}  // namespace bark

#endif // NEURALNETWORK_BASED_HEURISTIC_HPP