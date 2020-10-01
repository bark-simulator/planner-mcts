# Copyright (c) 2020 Julian Bernhard
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

import math
import itertools
import numpy as np
import logging
logging.basicConfig()
logging.getLogger().setLevel(logging.INFO)

from bark.core.models.behavior import *
from bark.core.models.behavior.risk_calculation import *
from bark.runtime.commons.parameters import ParameterServer
from bark_mcts.models.behavior.hypothesis.behavior_space.knowledge_function_definitions import *

class BehaviorSpace:
  def __init__(self, params):
    self._params = params.AddChild("BehaviorSpace")
    self._behavior_space_definition = self._params.AddChild("Definition")
    self._config_behavior_space()

  def sample_behavior_parameters(self, random_state = None):
    self._sampling_parameters = self._params.AddChild("Sampling")
    random_seed = self._sampling_parameters["RandomSeed", "Seed for parameter sampling", 1000]
    self.random_state = np.random.RandomState(random_seed)
    if random_state:
      self.random_state = random_state

    partitioned, partitioned_behavior_space_range_params = self._check_and_apply_partitioning(
                self._behavior_space_range_params, self._sampling_parameters)
    sampling_region_boundaries = None
    behavior_space_range_params = self._behavior_space_range_params
    if partitioned:
      behavior_space_range_params = partitioned_behavior_space_range_params
      sampling_region_boundaries, _ = \
          self._divide_distribution_ranges_and_fixed(partitioned_behavior_space_range_params)

    mean_space_params, knowledge_probability, \
      importance_sampling_probability = \
           self._sample_mean_params_from_prior_knowledge_function(behavior_space_range_params, \
                                                    sampling_region_boundaries)
    return self._sample_param_variations_from_param_means(mean_space_params, behavior_space_range_params, \
          self._sampling_parameters), self.model_type, knowledge_probability, \
          importance_sampling_probability

  def create_hypothesis_set_fixed_split(self, split):
    hypothesis_parameters = self.get_default_hypothesis_parameters()
    # todo set parameters
    def replace_partitioning(partition_parameters):
      for split_param, partition_num in partition_parameters.store.items():
        if isinstance(partition_num, ParameterServer):
          replace_partitioning(partition_num)
        elif not "Distribution" in split_param:
          logging.error("None distribution param type specified for hypothesis splitting.")
        else:
          partition_parameters[split_param] = split # only a single hypothesis covers whole distribution

    replace_partitioning(hypothesis_parameters.AddChild("Partitions"))
    return self.create_hypothesis_set(hypothesis_parameters)

  def create_cover_hypothesis(self):
    return self.create_hypothesis_set_fixed_split(split=1)

  def create_multiple_hypothesis_sets(self, splits):
    hypothesis_set_collection = {}
    for split in splits:
      hypothesis_set_collection[split] = \
             self.create_hypothesis_set_fixed_split(split=split)
    return hypothesis_set_collection

  def get_default_hypothesis_parameters(self):
    hypothesis_parameters = self._params.AddChild("Hypothesis")
    _ = hypothesis_parameters["RandomSeed", "Seed for hypothesis", 1000]
    hypothesis_model_type = hypothesis_parameters["HypothesisModel", "", "BehaviorHypothesisIDM"]
    _, defaults_hypothesis_model = self._model_from_model_type(hypothesis_model_type, ParameterServer())
    if len(hypothesis_parameters[hypothesis_model_type].ConvertToDict()) == 0:
      hypothesis_parameters[hypothesis_model_type] = defaults_hypothesis_model[hypothesis_model_type]
    partition_parameters = hypothesis_parameters.AddChild("Partitions")

    def add_default_partition_params(range_params, part_params):
      for param, value in range_params.store.items():
        if isinstance(value, ParameterServer):
          add_default_partition_params(self._behavior_space_range_params[param], part_params[param])
        if "Distribution" in param:
          _ = part_params[param, "Number of partitions", 1]

    def clean_default_partition_params(part_params):
      for param, value in part_params.store.copy().items():
        if isinstance(value, ParameterServer):
          if len(value.store) == 0:
            del part_params.store[param]
          else:
            clean_default_partition_params(value)

    add_default_partition_params(self._behavior_space_range_params, partition_parameters)
    clean_default_partition_params(partition_parameters)
    _ = hypothesis_parameters["HypothesisModel", "Model used as behavior model for hypothesis", "BehaviorHypothesisIDM"]
    return hypothesis_parameters.clone()

  def _get_param_range_partition(self, param_range, partition_num, which_partition):
    param_range_width = param_range[1] - param_range[0]
    lower_bound = param_range[0] + float(which_partition)*param_range_width/partition_num
    upper_bound = param_range[0] + float(which_partition+1)*param_range_width/partition_num
    return [lower_bound, upper_bound]

  def create_hypothesis_set(self, hypothesis_parameters=None):
    hypothesis_parameters = hypothesis_parameters or self.get_default_hypothesis_parameters()
    partition_parameters = hypothesis_parameters.AddChild("Partitions")
    seed = hypothesis_parameters["RandomSeed"]
    hypothesis_model_type = hypothesis_parameters["HypothesisModel"]
    param_partitions = []
    param_keys = []
    def fill_param_partitions(partition_parameters, range_params, key_prefix=None):
      for split_param, partition_num in partition_parameters.store.items():
        if isinstance(partition_num, ParameterServer):
          prefix = key_prefix + "::" + split_param if key_prefix else split_param
          fill_param_partitions(partition_num, range_params[split_param], prefix)
        elif not "Distribution" in split_param:
          logging.error("None distribution param type specified for hypothesis splitting.")
        else:
          param_keys.append(key_prefix + "::" + split_param)
          param_range = range_params[split_param]
          # uniform distribution type
          if len(param_range) == 2:
            partitions = []
            for idx in range(0, partition_num):
                partitions.append(self._get_param_range_partition(param_range, partition_num, idx))
            param_partitions.append(partitions)
          # distribution type fixed value
          elif len(param_range) == 1:
            partition = [param_range] 
            param_partitions.append(partition)

    fill_param_partitions(partition_parameters, self._behavior_space_range_params)
    hypotheses_partitions = list(itertools.product(*param_partitions))


    hypothesis_set = []
    hypothesis_set_params = []
    for hypotheses_partition in hypotheses_partitions:
      model_params = self._behavior_space_range_params.clone()
      for param_idx in range(0, len(hypotheses_partition)):
        # overwrite range parameter by deleting child
        distribution_params = model_params.AddChild(param_keys[param_idx], delete = param_idx == 0)
        # uniform distribution for this dimension of behavior space
        if len(hypotheses_partition[param_idx]) == 2:
          distribution_params["DistributionType"] = "UniformDistribution1D"
          distribution_params["RandomSeed"] = seed
          distribution_params["LowerBound"] = hypotheses_partition[param_idx][0]
          distribution_params["UpperBound"] = hypotheses_partition[param_idx][1]
        # fixed value distribution for this part of behavior space
        elif len(hypotheses_partition[param_idx])==1:
          distribution_params["DistributionType"] = "FixedValue"
          distribution_params["FixedValue"] = [hypotheses_partition[param_idx][0]]
      model_params[hypothesis_model_type] = hypothesis_parameters.AddChild(hypothesis_model_type).clone()
      param_server_behavior = ParameterServer(json = model_params.ConvertToDict(), log_if_default=True)
      hypothesis_behavior, _ = \
            self._model_from_model_type(hypothesis_model_type, param_server_behavior)
      hypothesis_set.append(hypothesis_behavior)
      hypothesis_set_params.append(param_server_behavior)
    return hypothesis_set, hypothesis_set_params

  def _config_behavior_space(self):
    self.model_type = self._behavior_space_definition["ModelType", "Model type over which behavior space is defined", \
        "BehaviorIDMStochastic"]

    # Space Boundary Parameters
    self._behavior_space_range_params = self._behavior_space_definition.AddChild("SpaceBoundaries")
    model_params = ParameterServer()
    _, _ = self._model_from_model_type(self.model_type, model_params)

    def replace_with_ranges(model_params, space_boundary_params):
        for key, value in model_params.store.items():
          if "Distribution" in key:
              space_boundary_params[key] = space_boundary_params[key, "Range for this distribution", [3.0, 4.0]]
              continue
          elif isinstance(value, ParameterServer):
            replace_with_ranges(value, space_boundary_params[key])
          else:
            space_boundary_params[key] = space_boundary_params[key, "Value", value]

    replace_with_ranges(model_params, self._behavior_space_range_params)

    # Prior Knowledge Parameters
    self._prior_knowledge_function_params = self._behavior_space_definition.AddChild("PriorKnowledgeFunction")
    prior_knowledge_definition_name = self._prior_knowledge_function_params["FunctionDefinition",
             "Specifies which class derived of KnowledgeFunctionDefinition should be used to define priior knowledge", \
                "TruncatedNormalKnowledgeFunctionDefinition"]
    ranges, _ = self._divide_distribution_ranges_and_fixed(self._behavior_space_range_params)
    self._prior_knowledge_region = PriorKnowledgeRegion(ranges)
    self._prior_knowledge_function_definition = eval("{}(self._prior_knowledge_function_params, \
                          self._prior_knowledge_region)".format(prior_knowledge_definition_name))
    self._prior_knowledge_function = PriorKnowledgeFunction(self._prior_knowledge_region, 
                                                    self._prior_knowledge_function_definition,
                                                    self._prior_knowledge_function_params)
    self._scenario_risk_function = None
  def _divide_distribution_ranges_and_fixed(self, behavior_space_range_params):
    def filter_distributions(dct):
      found_ranges = {}
      found_fixed = {}
      for key, val in dct.items():
        if "Distribution" in key:
          if len(val) == 2:
            found_ranges[key] = tuple(val)
          else:
            found_fixed[key] = val
        elif isinstance(val, dict):
          found_sub_range, found_sub_fixed = filter_distributions(val)
          for k, v in found_sub_range.items():
            found_ranges["{}::{}".format(key, k)] = v
          found_fixed[key] = {}
          for k, v in found_sub_fixed.items():
            found_fixed[key][k] = v
      return found_ranges, found_fixed
    return filter_distributions(behavior_space_range_params.ConvertToDict())

  def _model_from_model_type(self, model_type, params):
    bark_model = eval("{}(params)".format(model_type))
    return bark_model, params

  def get_prior_knowledge_function(self):
    return self._prior_knowledge_function 

  def get_scenario_risk_function(self):
    if not self._scenario_risk_function:
      scenario_risk_function_params = self._behavior_space_definition.AddChild("ScenarioRiskFunctionFunction")
      scenario_risk_func_def_name = scenario_risk_function_params["FunctionName", "Name of scenario risk function definition", "LinearKnowledgeFunctionDefinition"]
      scenario_risk_function_def = eval("{}(self._prior_knowledge_region, scenario_risk_function_params)".format(scenario_risk_func_def_name))

      self._scenario_risk_function = self._prior_knowledge_function.CalculateScenarioRiskFunction(
                              scenario_risk_function_def)
    return self._scenario_risk_function

  def _sample_mean_params_from_prior_knowledge_function(self, behavior_space_range_params, sample_region):
    mean_params = behavior_space_range_params.clone()
    sampled_dist_params = self._prior_knowledge_function_definition.Sample(sample_region, self.random_state)
    for param_sampled_hierarchy, param_value in sampled_dist_params[0].items():
      hierarchy_levels = param_sampled_hierarchy.split("::")
      current_child = mean_params
      for idx, hierarchy_level in enumerate(hierarchy_levels):
        if idx < len(hierarchy_levels) - 1:
          current_child = current_child[hierarchy_level]
        else:
          current_child[hierarchy_level] = param_value
    return mean_params, sampled_dist_params[1], sampled_dist_params[2]

  def _sample_mean_params_from_uniform_function(self):
    #todo
    pass

  def _check_and_apply_partitioning(self, behavior_space_range_params, sampling_params):
    partitioned_params = behavior_space_range_params.clone()

    def recursive_check_and_apply_partitioning(current_param_level, current_sampling_params):
      partitioned = False
      for key, value in current_param_level.store.items():
        if "Distribution" in key:
          parameter_range = value
          if len(parameter_range) > 1:
            partitions = current_sampling_params[key]["Partitions", "Into how many equal parameter range partitions is this range partitioned", None]
            if partitions :
              selected_partition = current_sampling_params[key]["SelectedPartition", "Which of these partitions is selected for sampling", 0]
              parameter_range = self._get_param_range_partition(parameter_range, partitions, selected_partition)
              current_param_level[key] = parameter_range
              partitioned = True
        elif isinstance(value, ParameterServer):
          partitioned = partitioned or recursive_check_and_apply_partitioning(current_param_level[key], current_sampling_params[key])
      return partitioned

    partitioned = recursive_check_and_apply_partitioning(partitioned_params, sampling_params)
    return partitioned, partitioned_params

  def _sample_param_variations_from_param_means(self, mean_space_params, behavior_space_range_params, sampling_params):
    """
    searches through param server to find distribution keys,
    adds by default to all distribution types a range parameter
    """
    param_dict = ParameterServer(log_if_default=True)
    for key, sampled_value in mean_space_params.store.items():
      child = sampling_params[key]
      param_range = behavior_space_range_params[key]
      if "Distribution" in key:
        distribution_type = sampling_params[key]["DistributionType", "Distribution type for sampling", "UniformDistribution1D"]
        if "Uniform" in distribution_type:
          param_dict[key] = self._sample_uniform_dist_params(sampled_value, param_range, child)
        elif "Normal" in distribution_type:
          param_dict[key] = self._sample_normal_dist_params(sampled_value, param_range, child)
        elif "Fixed" in distribution_type:
          param_dict[key] = self._get_fixed_dist_params(sampled_value, param_range, child)
      elif isinstance(sampled_value, ParameterServer):
        param_dict[key] = self._sample_param_variations_from_param_means(sampled_value, param_range, child)
      else:
        param_dict[key] = self._sample_non_distribution_params(sampled_value, param_range, child)
      if len(child.store) == 0:
        del sampling_params[key]
    return param_dict

  def _sample_non_distribution_params(self, sampled_mean, range, sampling_params):
    param_sampled = None
    if isinstance(range, list):
      if len(range) > 1:
        param_sampled = self.random_state.uniform(range[0], range[1])
      else:
        param_sampled = range[0]
    else:
      param_sampled = range
    return param_sampled

  def _sample_uniform_dist_params(self, sampled_mean, range, sampling_params):
    uni_width = sampling_params["Width", "What minimum and maximum width should sampled distribution have", [0.1, 0.3]]

    width = self.random_state.uniform(uni_width[0], uni_width[1])
    lower_offset = min(sampled_mean - range[0], max( width/2.0, width - (range[1] - sampled_mean)))
    lower_bound = sampled_mean - lower_offset
    upper_bound = sampled_mean + (width - (sampled_mean - lower_bound))

    sampled_params = ParameterServer(log_if_default = True)
    sampled_params["DistributionType"] = "UniformDistribution1D"
    sampled_params["LowerBound"] = lower_bound
    sampled_params["UpperBound"] = upper_bound
    sampled_params["RandomSeed"] = sampling_params["RandomSeed", "Seed for stochastic behavior", 1000]
    return sampled_params

  def _get_fixed_dist_params(self, sampled_mean, range, sampling_params):
    sampled_params = ParameterServer(log_if_default = True)
    sampled_params["DistributionType"] = "FixedValue"
    sampled_params["FixedValue"] = sampled_mean
    return sampled_params

  def _sample_normal_dist_params(self, sampled_mean, range, sampling_params):
    std_range = sampling_params["StdRange", "What minimum and maximum standard deviation should sampled distribution have", [0.1, 0.3]]

    std = self.random_state.uniform(std_range[0], std_range[1])
    mean = sampled_mean

    sampled_params = ParameterServer(log_if_default = True)
    sampled_params["DistributionType"] = "NormalDistribution1D"
    sampled_params["Mean"] = mean
    sampled_params["StdDev"] = std
    sampled_params["RandomSeed"] = sampling_params["RandomSeed", "Seed for stochastic behavior", 1000]
    return sampled_params
