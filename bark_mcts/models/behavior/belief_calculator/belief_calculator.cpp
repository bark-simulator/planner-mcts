#include "bark_mcts/models/behavior/belief_calculator/belief_calculator.hpp"
#define MCTS_EXPECT_TRUE(cond) BARK_EXPECT_TRUE(cond)
#include "bark_mcts/models/behavior/mcts_state/mcts_state_hypothesis.hpp"
#include "bark_mcts/models/behavior/param_config/mcts_parameters_from_param_server.hpp"
#include "bark/world/observed_world.hpp"

namespace bark {
namespace models {
namespace behavior {

BeliefCalculator::BeliefCalculator(const commons::ParamsPtr& params,
                                const std::vector<BehaviorModelPtr>& behavior_hypothesis) :
                                behavior_hypotheses_(behavior_hypothesis),
                                belief_tracker_(),
                                last_mcts_hypothesis_state_(),
                                mcts_parameters_([&]() -> mcts::MctsParameters {
                                   mcts::MctsParameters parameters;
                                   parameters.hypothesis_belief_tracker = models::behavior::BeliefTrackerParametersFromParamServer(params);
                                   return parameters;
                                }()) {}

void BeliefCalculator::Reset() {
  belief_tracker_ = std::make_shared<mcts::HypothesisBeliefTracker>(
            mcts_parameters_);
}

void BeliefCalculator::BeliefUpdate(
        const world::ObservedWorld& observed_world) {
  ObservedWorldPtr mcts_observed_world =
      std::dynamic_pointer_cast<ObservedWorld>(observed_world.Clone());

  if(!belief_tracker_) {
    Reset();
  }

  // Define initial mcts state
  StateParameters state_parameters;
  auto mcts_hypothesis_state_ptr = std::make_shared<MctsStateHypothesis<>>(
                                mcts_observed_world, 
                                false, 
                                0, 
                                0.0, 
                                belief_tracker_->sample_current_hypothesis(), // pass hypothesis reference to states
                                behavior_hypotheses_,
                                nullptr,
                                0,
                                state_parameters);

  if(!last_mcts_hypothesis_state_) {
    belief_tracker_->belief_update(*mcts_hypothesis_state_ptr, *mcts_hypothesis_state_ptr);
  } else {
    belief_tracker_->belief_update(*last_mcts_hypothesis_state_, *mcts_hypothesis_state_ptr);
  }
  last_mcts_hypothesis_state_ = mcts_hypothesis_state_ptr;
}

}  // namespace behavior
}  // namespace models
}  // namespace bark