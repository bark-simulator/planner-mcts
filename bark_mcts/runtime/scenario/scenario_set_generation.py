# Copyright (c) 2020 Julian Bernhard

# This software is released under the MIT License.
# https://opensource.org/licenses/MIT


class ScenarioSetGeneration:
  def __init__(self, base_params):
    self._base_params = base_params
    self._set_params_list = []

  def AddVariation(self, params_list, variations_list, param_root=None):
    # params_list: list of parameters in format ["child1::child2::param1", "child2::child1::param3"]
    # variations_list: list of variations, each variation is applied to both params at same time, e.g. [2.0, 4.0]
    new_params_list = []
    if not self._set_params_list:
      self._set_params_list.append(self._base_params.clone())
    for params in self._set_params_list:
      for variation in variations_list:
        new_params = params.clone()
        for param in params_list:
          if param_root:
            delim="::"
            param = "{}{}{}".format(param_root, delim, param)
          new_params[param] = variation
        new_params_list.append(new_params)
    self._set_params_list = new_params_list

  def GetSets(self, set_base_name):
    scenario_set_dict = {}
    for idx, params in enumerate(self._set_params_list):
      set_name = "{}_{}".format(set_base_name, idx)
      params["Scenario"]["Generation"]["SetName"] = set_name
      scenario_set_dict[set_name] = params 
    return scenario_set_dict


class BehaviorHypothesisScenarioSetGeneration(ScenarioSetGeneration):
  def __init__(self, base_params, partition_specs, hierarchy_param_name):
    super().__init__(base_params)
    self._hierarchy_param_name = hierarchy_param_name
    self._Partition(partition_specs)

  def _Partition(self, partition_specs):
    # first set the partitions correctly in the base parameters
    for partition_param, partition_spec in partition_specs.items(): 
      self._base_params["{}::{}::Partitions".format(self._hierarchy_param_name, \
            partition_param)] = partition_spec

    # then create the hypothesis scenario sets by adding variations for each spec
    for partition_param, partition_spec in partition_specs.items():
      if partition_spec:
        self.AddVariation(["{}::{}::SelectedPartition".format(self._hierarchy_param_name, \
              partition_param)], list(range(0, partition_spec)))

