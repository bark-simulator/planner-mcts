





std::vector<AgentIdx> BehaviorMCTS::get_agent_id_map (
    const world::ObservedWorld &observed_world) const {
  world::AgentMap agent_map = observed_world.GetOtherAgents();
  std::vector<mcts::AgentIdx> agent_ids(agent_map.size() + 1 - prediction_settings_.other_agents_.size());
  agent_ids[0] = observed_world.GetEgoAgent()->GetAgentId();
  size_t i = 1;
  for (const auto &agent : agent_map) {
    if(prediction_settings_.other_agents_.count(agent.first) == 0) {
      agent_ids[i] = agent.first;
      ++i;
    }
  }
  return agent_ids;
}