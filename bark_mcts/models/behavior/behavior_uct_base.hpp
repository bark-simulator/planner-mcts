// Copyright (c) 2020 Julian Bernhard
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef MODULES_MODELS_BEHAVIOR_BEHAVIOR_UCT_BASE_HPP_
#define MODULES_MODELS_BEHAVIOR_BEHAVIOR_UCT_BASE_HPP_

#include <memory>
#include "mcts/mcts.h"
#include "mcts/mcts_parameters.h"
#include "bark/models/behavior/behavior_model.hpp"
#include "bark/models/behavior/motion_primitives/motion_primitives.hpp"

#include "bark_mcts/models/behavior/mcts_state/mcts_state_base.hpp"

namespace bark {
namespace world {
class ObservedWorld;
}
namespace models {
namespace behavior {

class BehaviorUCTBase : public BehaviorModel {
 public:
  explicit BehaviorUCTBase(const commons::ParamsPtr& params);
  virtual ~BehaviorUCTBase() {}

  BehaviorMotionPrimitivesPtr GetEgoBehavior() const { return ego_behavior_model_; }

  using BarkMctsEdgeInfo = mcts::MctsEdgeInfo<Trajectory>;
  std::vector<BarkMctsEdgeInfo> GetLastMctsEdgeInfo() const {return mcts_edge_infos_; };
  void SetLastMctsEdgeInfo(const std::vector<BarkMctsEdgeInfo>& mcts_edge_infos)  {
        mcts_edge_infos_ = mcts_edge_infos; }

  template< class Mcts, class State>
  static std::vector<BarkMctsEdgeInfo> ExtractMctsEdgeInfo(Mcts& mcts, unsigned int max_depth);

  std::string GetPrimitiveName(mcts::ActionIdx action) const; 
    

 protected:
  BehaviorMotionPrimitivesPtr ego_behavior_model_;

  // PARAMETERS
  const mcts::MctsParameters mcts_parameters_;
  bool dump_tree_;
  bool extract_edge_info_;
  unsigned int max_extraction_depth_;
  double prediction_time_span_;
  StateParameters mcts_state_parameters_;
  std::vector<BarkMctsEdgeInfo> mcts_edge_infos_;
};

template< class Mcts, class State>
std::vector<BehaviorUCTBase::BarkMctsEdgeInfo> BehaviorUCTBase::ExtractMctsEdgeInfo(Mcts& mcts, unsigned int max_depth) {
  const std::function<Trajectory(const State&, 
                        const State&,
                        const mcts::AgentIdx&)> edge_info_extractor = [](const State& start_state, 
                                        const State& end_state,
                                        const mcts::AgentIdx& agent_idx) {
        return end_state.get_observed_world().GetAgents().at(agent_idx)->GetExecutionTrajectory();                                  
    };
  return mcts.visit_mcts_tree_edges(edge_info_extractor, max_depth);
}


}  // namespace behavior
}  // namespace models
}  // namespace bark

#endif  // MODULES_MODELS_BEHAVIOR_BEHAVIOR_UCT_BASE_HPP_