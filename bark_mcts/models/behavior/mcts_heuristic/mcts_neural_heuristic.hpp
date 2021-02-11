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

using bark_ml::lib_fqf_iqn_qrdqn::ModelLoader;

namespace mcts {
class CostConstrainedStatistic;
}

namespace bark {
namespace models {
namespace behavior {

class MctsNeuralHeuristic :  public mcts::Heuristic<MctsNeuralHeuristic>
{
public:
    MctsNeuralHeuristic(const mcts::MctsParameters& mcts_parameters) :
            mcts::Heuristic<MctsNeuralHeuristic>(mcts_parameters), 
            model_loader_(),
            observer_() {}

    void Initialize(const bark_ml::observers::ObserverPtr& observer,
                 const std::string& model_file_name) {
        model_loader_ = std::make_unique<ModelLoader>();
        observer_ = observer;
          if(!model_loader_->LoadModel(model_file_name)) {
            LOG(FATAL) << "Error during loading of model filename: " << model_file_name;
          }
    }

    template<class S, class SE, class SO, class H>
    std::pair<SE, std::unordered_map<mcts::AgentIdx, SO>>
             calculate_heuristic_values(const std::shared_ptr<mcts::StageNode<S,SE,SO,H>> &node) {
        using mcts::operator+=;
        namespace chr = std::chrono;
        std::shared_ptr<S> state = node->get_state()->clone();
        auto action_returns = mcts::ActionMapping(state->get_num_actions(state->get_ego_agent_idx()), 0.0); 
        auto action_costs = mcts::ActionMapping(state->get_num_actions(state->get_ego_agent_idx()),
                                                 mcts::EgoCosts(state->get_num_costs(), 0.0)); 
        auto action_executed_step_lengths = mcts::ActionMapping(state->get_num_actions(state->get_ego_agent_idx()),
                                             state->get_execution_step_length());  
        auto other_accum_rewards = mcts::AgentMapping(state->get_other_agent_idx(), 0.0); 


        const auto observed_nn_state = observer_->Observe(state->get_observed_world());
        std::vector<float> observed_vector(observed_nn_state.data(), observed_nn_state.data()
                                             + observed_nn_state.rows() * observed_nn_state.cols());
        const auto nn_output = model_loader_->Inference(observed_vector);
        
        // generate an extra node statistic for each agent
        SE ego_heuristic(0, node->get_state()->get_ego_agent_idx(), mcts_parameters_);
        if constexpr(std::is_same<SE, mcts::CostConstrainedStatistic>::value) {
          ego_heuristic.set_heuristic_estimate(action_returns, action_costs, action_executed_step_lengths);
          //VLOG_EVERY_N(6, 10) << "accum cost = " << action_costs << ", heuristic_step_length = " << action_executed_step_lengths;
          } else {
          ego_heuristic.set_heuristic_estimate(action_returns, action_costs); 
        }
        std::unordered_map<mcts::AgentIdx, SO> other_heuristic_estimates;
        mcts::AgentIdx reward_idx=1;
        mcts::EgoCosts mean_cost(state->get_num_costs(), 0.0);
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
            other_heuristic_estimates.insert(std::pair<mcts::AgentIdx, SO>(agent_idx, statistic));
            reward_idx++;
        }
        return std::pair<SE, std::unordered_map<mcts::AgentIdx, SO>>(ego_heuristic, other_heuristic_estimates);
    }

    private:
        std::unique_ptr<bark_ml::lib_fqf_iqn_qrdqn::ModelLoader> model_loader_;
        std::shared_ptr<bark_ml::observers::BaseObserver> observer_;
};

}  // namespace behavior
}  // namespace models
}  // namespace bark

#endif