# Copyright (c) 2020 Julian Bernhard

# This software is released under the MIT License.
# https://opensource.org/licenses/MIT


class ConfigureScenarioSets:
  def __init__(self, base_params):
    self._base_params = base_params
    self._set_params_list = [self._base_params.clone()]
    self._self_source_sink_root ="Scenario::Generation::ConfigurableScenarioGeneration::SinksSources"

  def SetSinkSourceParams(params, value):
    sink_sources = self._base_params.AddChild(self._self_source_sink_root)
    for sink_source in sinks_sources:
      sink_source_param_server = ParamServer(json=sink_source)
      sink_source_param_server


  def AddVariation(self, params_list, variations_list):
    # params_list: list of parameters in format ["child1::child2::param1", "child2::child1::param3"]
    # variations_list: list of variations, each variation is applied to both params at same time, e.g. [2.0, 4.0]
    new_params_list = []
    for params in self._set_params_list:
      for variation in variations_list:
        new_params = params.Clone()
        for param in params_list:
          new_params[param] = variation
        new_params_list.append(new_params)
    self._set_params_list = new_params_list

  def GenerateSets(self, set_base_name):
    scenario_set_dict = {}
    for idx, params in enumerate(self._set_params_list):
      scenario_set_dict["{}_{}".format(set_base_name, idx)] = params 
    return scenario_set_dict


class BehaviorHypothesisScenarioSetGeneration(ScenarioSetGeneration):
  def __init__(self, base_params, partition_specs):
    super().__init__(base_params)
    self._Partition(partition_specs)

  def _Partition(self, partition_specs):
    # first set the partitions correctly in the base parameters
    for partition_spec in partition_specs: 
      distribution_params = self._base_params.AddChild("{}::{}".format(self._behavior_space_root_params_name, \
            partition_spec))
      distribution_params["Partition"] = partition_spec

    # then create the hypothesis scenario sets by adding variations for each spec
    for partition_spec in partition_specs:
      sinks_sources = 
      self.AddVariation(["{}::{}::SelectedPartition".format(self._behavior_space_root_params_name, \
            partition_spec)], list(range(0, partition_spec)))

