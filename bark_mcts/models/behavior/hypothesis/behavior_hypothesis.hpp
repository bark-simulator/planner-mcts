// Copyright (c) 2020 Julian Bernhard
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef MODULES_MODELS_BEHAVIOR_BEHAVIOR_HYPOTHESIS_HPP_
#define MODULES_MODELS_BEHAVIOR_BEHAVIOR_HYPOTHESIS_HPP_

#include "bark/models/behavior/behavior_model.hpp"
#include "bark/commons/distribution/distribution.hpp"

#include "bark_mcts/models/behavior/risk_calculation/prior_knowledge_region.hpp"

namespace bark {
namespace world {
namespace objects {
class Agent;
typedef std::shared_ptr<Agent> AgentPtr;
typedef unsigned int AgentId;
}  // namespace objects
class ObservedWorld;
}  // namespace world
namespace models {
namespace behavior {


class BehaviorHypothesis : public virtual BehaviorModel, public risk_calculation::KnowledgeRegion {
  public:
    BehaviorHypothesis(const commons::ParamsPtr& params) : BehaviorModel(params) {}
    virtual ~BehaviorHypothesis() {}

    virtual bark::commons::Probability GetProbability(const Action& action,
                             const world::ObservedWorld& observed_world,
                             const world::objects::AgentId& agent_id) const = 0;

    bark::commons::Probability ParameterSamplingProbability() const { return 1.0 / risk_calculation::CalculateRegionBoundariesArea(this->GetDefinition()); }

    virtual void ChangeSeed(const bark::commons::RandomSeed& new_seed) = 0;

};

typedef std::shared_ptr<BehaviorHypothesis> BehaviorHypothesisPtr;


}  // namespace behavior
}  // namespace models
}  // namespace bark

#endif  // MODULES_MODELS_BEHAVIOR_BEHAVIOR_HYPOTHESIS_HPP_