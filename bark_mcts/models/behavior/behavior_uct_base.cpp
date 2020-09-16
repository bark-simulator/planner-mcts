#include "bark_mcts/models/behavior/behavior_uct_base.hpp"
#include "bark_mcts/models/behavior/param_config/mcts_parameters_from_param_server.hpp"
#include "bark_mcts/models/behavior/param_config/mcts_state_parameters_from_param_server.hpp"
#include "bark/models/behavior/motion_primitives/param_config/behav_macro_actions_from_param_server.hpp"

namespace bark {
namespace models {
namespace behavior {

BehaviorUCTBase::BehaviorUCTBase(
    const commons::ParamsPtr& params)
    : BehaviorModel(params),
      ego_behavior_model_(models::behavior::
          BehaviorMacroActionsFromParamServer(GetParams()
            ->AddChild("BehaviorUctBase")->AddChild("EgoBehavior"))),
      mcts_parameters_(models::behavior::MctsParametersFromParamServer(
          GetParams()->AddChild("BehaviorUctBase"))),
      dump_tree_(GetParams()->AddChild("BehaviorUctBase")->GetBool(
          "DumpTree",
          "If true, tree is dumped to dot file after planning", false)),
      extract_edge_info_(GetParams()->AddChild("BehaviorUctBase")->GetBool(
          "ExtractEdgeInfo",
          "If true, policy tree is maintained in each step", true)),
      prediction_time_span_(GetParams()->AddChild("BehaviorUctBase")
                                        ->AddChild("PredictionSettings")
                                        ->GetReal("TimeSpan",
          "Time in seconds agents are predicted ahead in each expansion and rollout step", 0.5f)),
      mcts_state_parameters_(MctsStateParametersFromParamServer(
          GetParams()->AddChild("BehaviorUctBase"))),
      mcts_edge_infos_() {}


}  // namespace behavior
}  // namespace models
}  // namespace bark