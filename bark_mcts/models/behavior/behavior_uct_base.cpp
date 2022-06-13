#include "bark_mcts/models/behavior/behavior_uct_base.hpp"
#include "bark_mcts/models/behavior/param_config/mcts_parameters_from_param_server.hpp"
#include "bark_mcts/models/behavior/param_config/mcts_state_parameters_from_param_server.hpp"
#include "bark/models/behavior/motion_primitives/param_config/behav_macro_actions_from_param_server.hpp"

namespace bark {
namespace models {
namespace behavior {

BehaviorUCTBase::BehaviorUCTBase(const commons::ParamsPtr& params, const UctBaseDebugInfos& base_debug_infos) :
                            BehaviorModel(params),
                            UctBaseDebugInfos(base_debug_infos), 
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
                            extract_state_info_(GetParams()->AddChild("BehaviorUctBase")->GetBool(
                            "ExtractStateInfo",
                            "If true, tree node information is maintained in each step", false)),
                            max_extraction_depth_(GetParams()->AddChild("BehaviorUctBase")
                                                                ->GetInt("MaxExtractionDepth",
                                "Max depth tree is extracted", 10)),
                            max_nearest_agents_(GetParams()->AddChild("BehaviorUctBase")
                                                                ->GetInt("MaxNearestAgents",
                                "Max sourrounding agents considered for tree search", 5)),
                            mcts_state_parameters_(MctsStateParametersFromParamServer(
                                GetParams()->AddChild("BehaviorUctBase"))),
                            constant_action_idx_(GetParams()->AddChild("BehaviorUctBase")
                                                                ->GetInt("ConstantActionIndex",
                                "Instead of applying the best motion index"
                                    "a specific index is applied if idx>= 0, serves for evaluation e.g. of beliefs", -1)) {}

BehaviorUCTBase::BehaviorUCTBase(
    const commons::ParamsPtr& params)
    : BehaviorUCTBase(params, UctBaseDebugInfos()) {}

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
    const auto nearest_agents = observed_world.GetNearestAgents(ego_position, max_nearest_agents_+1); // one for ego
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