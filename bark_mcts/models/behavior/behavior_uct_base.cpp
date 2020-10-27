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
      last_motion_idx_(),
      mcts_parameters_(models::behavior::MctsParametersFromParamServer(
          GetParams()->AddChild("BehaviorUctBase"))),
      dump_tree_(GetParams()->AddChild("BehaviorUctBase")->GetBool(
          "DumpTree",
          "If true, tree is dumped to dot file after planning", false)),
      extract_edge_info_(GetParams()->AddChild("BehaviorUctBase")->GetBool(
          "ExtractEdgeInfo",
          "If true, policy tree is maintained in each step", true)),
    max_extraction_depth_(GetParams()->AddChild("BehaviorUctBase")
                                        ->GetInt("MaxExtractionDepth",
          "Max depth tree is extracted", 10)),
    max_nearest_agents_(GetParams()->AddChild("BehaviorUctBase")
                                        ->GetInt("MaxNearestAgents",
          "Max sourrounding agents considered for tree search", 5)),
      prediction_time_span_(GetParams()->AddChild("BehaviorUctBase")
                                        ->AddChild("PredictionSettings")
                                        ->GetReal("TimeSpan",
          "Time in seconds agents are predicted ahead in each expansion and rollout step", 0.5f)),
      mcts_state_parameters_(MctsStateParametersFromParamServer(
          GetParams()->AddChild("BehaviorUctBase"))),
      mcts_edge_infos_() {}


std::string BehaviorUCTBase::GetPrimitiveName(mcts::ActionIdx action) const {
    auto macro_actions_model = std::dynamic_pointer_cast<BehaviorMPMacroActions>(ego_behavior_model_);
    if(macro_actions_model) {
      return macro_actions_model->GetMotionPrimitives().at(action)->GetName();
    } else {
      return "Unknown";
    }
}

ObservedWorldPtr BehaviorUCTBase::FilterAgents(const ObservedWorld& observed_world) const {
    const auto ego_position = observed_world.CurrentEgoPosition();
    const auto nearest_agents = observed_world.GetNearestAgents(ego_position, max_nearest_agents_);
    auto filtered_world = std::make_shared<ObservedWorld>(observed_world);
    filtered_world->ClearAgents();
    for(const auto& agent : nearest_agents) {
        filtered_world->AddAgent(agent.second);
    }
    filtered_world->UpdateAgentRTree();
    return filtered_world;
}


}  // namespace behavior
}  // namespace models
}  // namespace bark