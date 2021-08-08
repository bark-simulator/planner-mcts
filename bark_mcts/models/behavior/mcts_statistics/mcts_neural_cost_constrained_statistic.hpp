// Copyright (c) 2019 Julian Bernhard
// 
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.
// ========================================================

#ifndef UCT_NEURAL_COST_CONSTRAINED_STATISTIC_H
#define UCT_NEURAL_COST_CONSTRAINED_STATISTIC_H

#include "mcts/cost_constrained/cost_constrained_statistic.h"
#include "bark_ml/observers/base_observer.hpp"
#include "bark_ml/library_wrappers/lib_fqf_iqn_qrdqn/model_loader/model_loader.hpp"
#include "bark_ml/library_wrappers/lib_fqf_iqn_qrdqn/model/nn_to_value_converter/nn_to_value_converter.hpp"

using bark_ml::lib_fqf_iqn_qrdqn::ModelLoader;
using bark_ml::lib_fqf_iqn_qrdqn::ValueType;

namespace mcts {

inline std::unordered_map<mcts::ActionIdx, mcts::EgoCosts> ValueListToValueMaps(const std::vector<double>& l1,
                                                               const std::vector<double>& l2) {
  std::unordered_map<mcts::ActionIdx, mcts::EgoCosts> value_map;
  BARK_EXPECT_TRUE(l1.size() == l2.size());
  for(std::size_t action_idx = 0; action_idx < l1.size(); ++action_idx) {
    value_map[action_idx] = mcts::EgoCosts{l1.at(action_idx), l2.at(action_idx)};
  }                                        
  return value_map;
}

inline std::unordered_map<mcts::ActionIdx, double> ValueListToValueMap(const std::vector<double>& l1) {
  std::unordered_map<mcts::ActionIdx, double> value_map;
  for(std::size_t action_idx = 0; action_idx < l1.size(); ++action_idx) {
    value_map[action_idx] = l1.at(action_idx);
  }                                        
  return value_map;
}

// A upper confidence bound implementation
class NeuralCostConstrainedStatistic : public mcts::NodeStatistic<NeuralCostConstrainedStatistic>
{
public:
    MCTS_TEST

    NeuralCostConstrainedStatistic(ActionIdx num_actions, AgentIdx agent_idx, const MctsParameters & mcts_parameters) :
             mcts::NodeStatistic<NeuralCostConstrainedStatistic>(num_actions, agent_idx, mcts_parameters),
             cost_constrained_statistic_(num_actions, agent_idx, mcts_parameters),
             initialized_(false)
             {
             }

    ~NeuralCostConstrainedStatistic() {};

    template <class S>
    ActionIdx choose_next_action(const S& state) {
      if(!initialized_) {
        initialize_from_neural_model(state);
        initialized_ = true;
      }
      set_step_length(state.get_execution_step_length())
      if( cost_constrained_statistic_.policy_is_ready())
      {
        // Expansion policy does consider node counts
        return greedy_policy(cost_constrained_statistic_.get_kappa(), cost_constrained_statistic_.get_action_filter_factor()).first;
      } else {
          // Select randomly an unexpanded action
          auto sampled_action = sample_policy(cost_constrained_statistic_.get_exploration_policy).first;
          return sampled_action;
      }
    }

    Policy get_policy() const {
      return cost_constrained_statistic_.greedy_policy(0.0f, cost_constrained_statistic_.get_action_filter_factor()).second;
    }

    ActionIdx get_best_action() const {
      return cost_constrained_statistic_.greedy_policy(0.0f, cost_constrained_statistic_.get_action_filter_factor()).first;
    }

    PolicySampled greedy_policy(const double kappa_local, const double action_filter_factor_local) const {
      return cost_constrained_statistic_.greedy_policy(kappa_local, action_filter_factor_local);
    }

    Cost calc_updated_constraint_based_on_policy(const PolicySampled& policy, const Cost& current_constraint, const Cost& mean_step_cost) const {
      return cost_constrained_statistic_.calc_updated_constraint_based_on_policy(policy, current_constraint, mean_step_cost);
    }

    std::vector<Cost> calc_updated_constraints_based_on_policy(const PolicySampled& policy, const std::vector<Cost>& current_constraints) const {
      return cost_constrained_statistic_.calc_updated_constraints_based_on_policy(policy, current_constraints);
    }

    ActionIdx calculate_ucb_maximizing_action(const std::vector<ActionIdx>& allowed_actions, const double& kappa_local) const {
      return cost_constrained_statistic_.calculate_ucb_maximizing_action(allowed_actions, kappa_local);
    }


    void update_from_heuristic(const NodeStatistic<NeuralCostConstrainedStatistic>& heuristic_statistic)
    {
      cost_constrained_statistic_.update_from_heuristic(heuristic_statistic.impl().get_cost_constraint_node_statistic());
    }

    void update_statistic(const NodeStatistic<NeuralCostConstrainedStatistic>& changed_child_statistic) {
      cost_constrained_statistic_.collect(collected_reward_.second, collected_cost_.second, collected_reward_.first, 
                                          collected_action_transition_counts_);
      cost_constrained_statistic_.update_statistic(changed_child_statistic.impl().get_cost_constraint_node_statistic());
    }

    void set_heuristic_estimate(const Reward& accum_rewards, const EgoCosts& accum_ego_cost, double backpropated_step_length)
    {
      cost_constrained_statistic_.set_heuristic_estimate(accum_rewards, accum_ego_cost, backpropated_step_length);
    }

    void set_heuristic_estimate(const std::unordered_map<ActionIdx, Reward> &action_returns,
                                const std::unordered_map<ActionIdx, EgoCosts>& action_costs,
                                const std::unordered_map<ActionIdx, double>& backpropated_step_length)
    {
      cost_constrained_statistic_.set_heuristic_estimate(action_returns, action_costs, backpropated_step_length);
    }

    std::string print_node_information() const {
        return "";
    }

    static std::string print_policy(const Policy& policy) {
      std::stringstream ss;
      ss << "Policy: ";
      for (const auto& action_pair : policy) {
        ss << "P(a=" << action_pair.first << ") = " << action_pair.second << ", ";
      }
      return ss.str();
    }

    std::vector<mcts::Cost> expected_policy_cost(const Policy& policy) const {
      return cost_constrained_statistic_.expected_policy_cost(policy);
    }

    std::string print_edge_information(const ActionIdx& action) const {
        return cost_constrained_statistic_.print_edge_information(action);
    }

    unsigned get_num_costs() const { return cost_constrained_statistic_.get_num_costs(); }


    const UctStatistic& get_reward_statistic() const {
      return cost_constrained_statistic_.get_reward_statistic();
    }

    const UctStatistic& get_cost_statistic(const unsigned int& cost_index) const {
      return cost_constrained_statistic_.get_cost_statistic(cost_index);
    }

    std::string sprintf() const {
      return cost_constrained_statistic_.sprintf();
    }

    
    void merge_node_statistics(const std::vector<NeuralCostConstrainedStatistic>& statistics) {
      std::vector<CostConstrainedStatistic> statistics_cost;
      for(const auto stat : statistics) {
          statistics_cost.push_back(stat.get_cost_constraint_node_statistic());
      }
      cost_constrained_statistic_.merge_node_statistics(statistics_cost);
    }

    const CostConstrainedStatistic& get_cost_constraint_node_statistic() const { return cost_constrained_statistic_; }

    static void setup_neural_model(const bark_ml::observers::ObserverPtr& observer,
                 const std::string& model_file_name,
                 const bark_ml::lib_fqf_iqn_qrdqn::NNToValueConverterPtr& nn_to_value_converter) {
        model_loader_ = std::make_unique<ModelLoader>();
        observer_ = observer;
        nn_to_value_converter_ = nn_to_value_converter;
        if(!model_loader_->LoadModel(model_file_name)) {
          LOG(FATAL) << "Error during loading of model filename: " << model_file_name;
        }
    }

    template <class S>
    void initialize_from_neural_model(const S& state) {
        const auto observed_nn_state = observer_->Observe(state.impl().get_observed_world());
        std::vector<float> observed_vector(observed_nn_state.data(), observed_nn_state.data()
                                             + observed_nn_state.rows() * observed_nn_state.cols());
        const auto nn_output = model_loader_->Inference(observed_vector);
        const auto value_map = nn_to_value_converter_->ConvertToValueMap(nn_output);

        if(value_map.find(ValueType::Policy) != value_map.end()) {
            const auto policy = ValueListToValueMap(value_map.at(ValueType::Policy));
            cost_constrained_statistic_.set_exploration_policy(policy);
        }
        if(value_map.find(ValueType::EnvelopeRisk) != value_map.end() &&
            value_map.find(ValueType::CollisionRisk) != value_map.end() &&
            value_map.find(ValueType::Return) != value_map.end()) {

            const auto action_returns = ValueListToValueMap(value_map.at(ValueType::Return));
            const auto& action_envelope_risk = value_map.at(ValueType::EnvelopeRisk);
            const auto& action_collision_risk = value_map.at(ValueType::CollisionRisk);
            const auto action_costs = ValueListToValueMaps(action_envelope_risk, action_collision_risk);

            cost_constrained_statistic_.set_heuristic_estimate(action_returns, action_costs, std::unordered_map<ActionIdx, double>{});
        }
    }

private:
    CostConstrainedStatistic cost_constrained_statistic_;
    bool initialized_;
    static std::shared_ptr<bark_ml::lib_fqf_iqn_qrdqn::ModelLoader> model_loader_;
    static std::shared_ptr<bark_ml::observers::BaseObserver> observer_;
    static bark_ml::lib_fqf_iqn_qrdqn::NNToValueConverterPtr nn_to_value_converter_;
};



template <>
inline void NodeStatistic<NeuralCostConstrainedStatistic>::update_statistic_parameters(MctsParameters& parameters,
                                            const NeuralCostConstrainedStatistic& root_statistic,
                                            const unsigned int& current_iteration) {
    CostConstrainedStatistic::update_statistic_parameters(parameters,
                     root_statistic.get_cost_constraint_node_statistic(),
                     current_iteration);
}

template <>
inline MctsParameters NodeStatistic<NeuralCostConstrainedStatistic>::merge_mcts_parameters(std::vector<MctsParameters> parameters) {
    return CostConstrainedStatistic::merge_mcts_parameters(parameters);
}

} // namespace mcts

#endif