// Copyright (c) 2019 fortiss GmbH
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.


#ifndef MODULES_MODELS_BEHAVIOR_SINGLE_AGENT_MCTS_HPP_
#define MODULES_MODELS_BEHAVIOR_SINGLE_AGENT_MCTS_HPP_

#include "mcts/mcts_parameters.h"

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

    virtual modules::models::behavior::BehaviorModelPtr Clone() const;

  private:
    modules::world::prediction::PredictionSettings prediction_settings_;
    bool dump_tree_;

    // MCTS PARAMETERS
    mcts::MctsParameters mcts_parameters_;


};

inline modules::models::behavior::BehaviorModelPtr BehaviorUCTSingleAgent::Clone() const {
  return std::make_shared<BehaviorUCTSingleAgent>(*this);
}

}  // namespace behavior
}  // namespace models
}  // namespace modules

#endif  // MODULES_MODELS_BEHAVIOR_SINGLE_AGENT_MCTS_HPP_
