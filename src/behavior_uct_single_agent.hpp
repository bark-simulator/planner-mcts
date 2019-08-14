// Copyright (c) 2019 fortiss GmbH
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.


#ifndef MODULES_MODELS_BEHAVIOR_SINGLE_AGENT_MCTS_HPP_
#define MODULES_MODELS_BEHAVIOR_SINGLE_AGENT_MCTS_HPP_

#include "external/bark_project/modules/models/behavior/behavior_model.hpp"

namespace modules {
namespace world {
  class ObservedWorld;
}
namespace models {
namespace behavior {

class BehaviorUCTSingleAgent : public BehaviorModel {
 public:
   explicit BehaviorUCTSingleAgent(commons::Params *params) :
    BehaviorModel(params) {}

    virtual Trajectory Plan(float delta_time,
                 const world::ObservedWorld& observed_world) {return Trajectory();};

    virtual ~BehaviorUCTSingleAgent() {}

    virtual BehaviorModel *Clone() const;
};

inline BehaviorModel *BehaviorUCTSingleAgent::Clone() const {
  return new BehaviorUCTSingleAgent(*this);
}

}  // namespace behavior
}  // namespace models
}  // namespace modules

#endif  // MODULES_MODELS_BEHAVIOR_SINGLE_AGENT_MCTS_HPP_
