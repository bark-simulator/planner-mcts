# Copyright (c) 2020 Julian Bernhard
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

import math
import itertools
import numpy as np
import logging
import scipy.stats 
logging.basicConfig()
logging.getLogger().setLevel(logging.INFO)

from bark.core.models.behavior import *
from bark.core.models.behavior.risk_calculation import *
from bark.runtime.commons.parameters import ParameterServer

class SciPyKnowledgeFunctionDefinition(KnowledgeFunctionDefinition):
  def __init__(self, params, supporting_region):
    super().__init__(supporting_region, params.AddChild("TruncatedNormalKnowledgeFunctionDefinition"))
    self.SetDefaultDistributionParams(self.supporting_region.definition)

  def SetDefaultDistributionParams(self, supporting_region):
    raise NotImplementedError()

  def GetDistFunc(self, region_params, region_range, random_state=None):
    raise NotImplementedError()

  def GetDistFuncOfRegion(self, region_desc):
    random_state = np.random.RandomState(1000)
    return self.GetDistFunc(self.params[region_desc], self.supporting_region.definition[region_desc], random_state)

  def CalculateIntegral(self, region_boundaries):
    #todo improve with mean
    pdf_product = 1.0
    for reg_desc, reg_range in region_boundaries.items():
      reg_center = reg_range[1] - reg_range[0]
      pdf_val = self.GetDistFunc(self.params[reg_desc], reg_range).pdf(reg_center)
      pdf_product *= pdf_val
    return pdf_product*CalculateRegionBoundariesArea(region_boundaries)

  def _SampleFromRegion(self, sampling_region, random_state):
    sampled_values = {}
    total_values_prob = 1.0
    for reg_desc, reg_range in sampling_region.items():
      sampled_val = random_state.uniform(low=reg_range[0], high=reg_range[1])
      val_prob = self.GetDistFunc(self.params[reg_desc], reg_range, random_state).pdf(sampled_val)
      total_values_prob *= val_prob
      sampled_values[reg_desc] = sampled_val
    return sampled_values, total_values_prob, 1/CalculateRegionBoundariesArea(sampling_region)

  def _SampleFromDensity(self, random_state):
    sampled_values = {}
    total_values_prob = 1.0
    for reg_desc, reg_range in self.supporting_region.definition.items():
      dist_funct = self.GetDistFunc(self.params[reg_desc], reg_range, random_state)
      sampled_val = dist_funct.rvs(size=1)
      val_prob = dist_funct.pdf(sampled_val)
      total_values_prob *= val_prob[0]
      sampled_values[reg_desc] = sampled_val[0]
    return sampled_values, total_values_prob, total_values_prob

  def Sample(self, sampling_region, random_state):
    if sampling_region:
      return self._SampleFromRegion(sampling_region, random_state)
    else:
      return self._SampleFromDensity(random_state)


class UniformKnowledgeFunctionDefinition(SciPyKnowledgeFunctionDefinition):
  def __init__(self, *args, **kwargs):
    super().__init__(*args, **kwargs)

  def GetDistFunc(self, region_params, region_range, random_state=None):
    dist_func = scipy.stats.uniform(loc=region_range[0], scale=region_range[1] - region_range[0])
    dist_func.random_state = random_state
    return dist_func

  def SetDefaultDistributionParams(self, supporting_region):
    self._dist_funcs = {}
    for reg_desc, region in supporting_region.items():
        self.params[reg_desc] = None

class TruncatedNormalKnowledgeFunctionDefinition(SciPyKnowledgeFunctionDefinition):
  def __init__(self, *args, **kwargs):
    super().__init__(*args, **kwargs)

  def GetDistFunc(self, region_params, region_range, random_state=None):
    mean = region_params["Mean"]
    std = region_params["Std"]
    a, b = (region_range[0] - mean) / std, (region_range[1] - mean) / std
    dist_func = scipy.stats.truncnorm(a=a, b=b, loc=mean, scale=std)
    dist_func.random_state = random_state
    return dist_func

  def SetDefaultDistributionParams(self, supporting_region):
    self._dist_funcs = {}
    for reg_desc, region in supporting_region.items():
      _ = self.params[reg_desc]["Mean", "Mean of truncated normal", 5]
      _ = self.params[reg_desc]["Std", "Std of truncated normal", 1]