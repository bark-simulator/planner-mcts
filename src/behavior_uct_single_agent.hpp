// Copyright (c) 2019 fortiss GmbH
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.


#ifndef MODULES_MODELS_BEHAVIOR_SINGLE_AGENT_MCTS_HPP_
#define MODULES_MODELS_BEHAVIOR_SINGLE_AGENT_MCTS_HPP_

#include "modules/models/behavior/behavior_model.hpp"
#include "modules/world/prediction/prediction_settings.hpp"

namespace modules {
namespace world {
  class ObservedWorld;
}
namespace models {
namespace behavior {


class BehaviorUCTSingleAgent : public BehaviorModel {
 public:
   explicit BehaviorUCTSingleAgent(commons::Params *params);

    virtual Trajectory Plan(float delta_time,
                 const world::ObservedWorld& observed_world);

    virtual ~BehaviorUCTSingleAgent() {}

    virtual BehaviorModel *Clone() const;

  private:
    modules::world::prediction::PredictionSettings prediction_settings_;
    unsigned int max_num_iterations_;
    unsigned int max_search_time_;
    unsigned int random_seed_;
    bool dump_tree_;

    // MCTS PARAMETERS
    void update_mcts_parameters(); 
    double discount_factor_;
    double uct_exploration_constant_;

    double max_search_time_random_heuristic_;
    double max_number_iterations_random_heuristic_;

    double return_lower_bound_;
    double return_upper_bound_;
};

inline BehaviorModel *BehaviorUCTSingleAgent::Clone() const {
  return new BehaviorUCTSingleAgent(*this);
}

}  // namespace behavior
}  // namespace models
}  // namespace modules

#endif  // MODULES_MODELS_BEHAVIOR_SINGLE_AGENT_MCTS_HPP_
