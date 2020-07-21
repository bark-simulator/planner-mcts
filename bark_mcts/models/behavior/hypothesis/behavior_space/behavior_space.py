# Copyright (c) 2020 fortiss GmbH
#
# Author: Julian Bernhard
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
from bark.runtime.commons.parameters import ParameterServer

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
    return self._sample_params_from_param_ranges(self._behavior_space_range_params, \
                self._sampling_parameters), self.model_type


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

  def _model_from_model_type(self, model_type, params):
    bark_model = eval("{}(params)".format(model_type))
    return bark_model, params

  def _sample_params_from_param_ranges(self, space_params, sampling_params):
    """
    searches through param server to find distribution keys,
    adds by default to all distribution types a range parameter
    """

    param_dict = ParameterServer(log_if_default=True)
    for key, value in space_params.store.items():
      child = sampling_params[key]
      if "Distribution" in key:
        distribution_type = sampling_params[key]["DistributionType", "Distribution type for sampling", "UniformDistribution1D"]
        parameter_range = value
        if len(parameter_range) > 1:
          partitions = sampling_params[key]["Partitions", "Into how many equal parameter range partitions is this range partitioned", None]
          if partitions :
            selected_partition = sampling_params[key]["SelectedPartition", "Which of these partitions is selected for sampling", 0]
            parameter_range = self._get_param_range_partition(parameter_range, partitions, selected_partition)
        if "Uniform" in distribution_type and len(parameter_range) == 2:
          param_dict[key] = self._sample_uniform_dist_params(parameter_range, child)
        elif "Normal" in distribution_type and len(parameter_range) == 2:
          param_dict[key] = self._sample_normal_dist_params(parameter_range, child)
        elif "Fixed" in distribution_type or len(parameter_range) == 1:
          param_dict[key] = self._get_fixed_dist_params(parameter_range, child)
      elif isinstance(value, ParameterServer):
        param_dict[key] = self._sample_params_from_param_ranges(value, child)
      else:
        parameter_range = value
        param_dict[key] = self._sample_non_distribution_params(parameter_range, child)
      if len(child.store) == 0:
        del sampling_params[key]
    return param_dict

  def _sample_non_distribution_params(self, range, sampling_params):
    param_sampled = None
    if isinstance(range, list):
      if len(range) > 1:
        param_sampled = self.random_state.uniform(range[0], range[1])
      else:
        param_sampled = range[0]
    else:
      param_sampled = range
    return param_sampled

  def _sample_uniform_dist_params(self, range, sampling_params):
    uni_width = sampling_params["Width", "What minimum and maximum width should sampled distribution have", [0.1, 0.3]]

    lower_bound = self.random_state.uniform(range[0], range[1] - uni_width[0])
    upper_bound = self.random_state.uniform(lower_bound + uni_width[0], min(range[1], lower_bound + uni_width[1]))

    sampled_params = ParameterServer(log_if_default = True)
    sampled_params["DistributionType"] = "UniformDistribution1D"
    sampled_params["LowerBound"] = lower_bound
    sampled_params["UpperBound"] = upper_bound
    sampled_params["RandomSeed"] = sampling_params["RandomSeed", "Seed for stochastic behavior", 1000]
    return sampled_params

  def _get_fixed_dist_params(self, range, sampling_params):
    sampled_params = ParameterServer(log_if_default = True)
    sampled_params["DistributionType"] = "FixedValue"
    sampled_params["FixedValue"] = range
    return sampled_params

  def _sample_normal_dist_params(self, range, sampling_params):
    std_range = sampling_params["StdRange", "What minimum and maximum standard deviation should sampled distribution have", [0.1, 0.3]]

    std = self.random_state.uniform(std_range[0], std_range[1])
    mean = self.random_state.uniform(range[0], range[1])

    sampled_params = ParameterServer(log_if_default = True)
    sampled_params["DistributionType"] = "NormalDistribution1D"
    sampled_params["Mean"] = mean
    sampled_params["StdDev"] = std
    sampled_params["RandomSeed"] = sampling_params["RandomSeed", "Seed for stochastic behavior", 1000]
    return sampled_params
