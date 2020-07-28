# Copyright (c) 2019 Julian Bernhard
#
# This software is released under the MIT License.
# https://opensource.org/licenses/MIT

try:
  import debug_settings
except:
  pass


import unittest
import os
import numpy as np
from collections import defaultdict
from operator import itemgetter

from bark.runtime.commons.parameters import ParameterServer
from bark_mcts.models.behavior.hypothesis.behavior_space.behavior_space import BehaviorSpace
from bark.core.models.behavior.risk_calculation import *

class PyScenarioRiskFunctionTests(unittest.TestCase):
  def test_prior_knowledge_region_partition_1D(self):
    param_server = ParameterServer()
    prior_knowledge_region = PriorKnowledgeRegion({"1DDimensionName" : (-2.0, 5.0)})
    partitions = prior_knowledge_region.Partition(100)
    total_range = -2.0
    desired_width = 7.0/100.0
    for partition in partitions:
      region_range = partition.region_boundaries["1DDimensionName"]
      width = region_range[1] -region_range[0]
      self.assertAlmostEqual(width, desired_width, 4)
      total_range += width
    self.assertAlmostEqual(total_range, 5.0)


  def test_1D_functions_uniform(self):
    param_server = ParameterServer()
    prior_knowledge_region = PriorKnowledgeRegion({"1DDimensionName" : (-3.0, 7.0)})

    def knowledge_function_1D_uniform(region_boundaries):
      if not len(region_boundaries) == 1:
        raise ValueError("Only 1D knowledge function provided")
      region_range = region_boundaries["1DDimensionName"]
      uniform_prob = 1.0/10.0
      return uniform_prob*(region_range[1] - region_range[0])

    def scenario_risk_templ_func_1D(region_boundaries):
      if not len(region_boundaries) == 1:
        raise ValueError("Only 1D scenario risk function provided")
      # template function is a*x+ b then indefinite integral is a*x^2 + b*x
      region_range = region_boundaries["1DDimensionName"]
      uniform_prob = 0.5
      return uniform_prob*(region_range[1] - region_range[0])

    prior_knowledge_function = PriorKnowledgeFunction(prior_knowledge_region, 
                                                    knowledge_function_1D_uniform,
                                                    param_server)

    scenario_risk_function = prior_knowledge_function.CalculateScenarioRiskFunction(
                              scenario_risk_templ_func_1D)

    indef_int = lambda x : 1.0/10.0*0.5*x
    desired_normalization_const = 1.0/(indef_int(7.0)- indef_int(-3.0))
    self.assertAlmostEqual(scenario_risk_function.normalization_constant,
              desired_normalization_const, 5)

  def test_1D_functions_1uniform_1linear(self):
    param_server = ParameterServer()
    prior_knowledge_region = PriorKnowledgeRegion({"1DDimensionName" : (-3.0, 7.0)})

    def knowledge_function_1D_uniform(region_boundaries):
      if not len(region_boundaries) == 1:
        raise ValueError("Only 1D knowledge function provided")
      region_range = region_boundaries["1DDimensionName"]
      uniform_prob = 1.0/10.0
      return uniform_prob*(region_range[1] - region_range[0])

    def scenario_risk_templ_func_1D(region_boundaries):
      if not len(region_boundaries) == 1:
        raise ValueError("Only 1D scenario risk function provided")
      # template function is a*x+ b then indefinite integral is a*x^2 + b*x
      region_range = region_boundaries["1DDimensionName"]
      indef_int = lambda x : 2*x**2 + 3*x 
      return indef_int(region_range[1]) - indef_int(region_range[0])

    prior_knowledge_function = PriorKnowledgeFunction(prior_knowledge_region, 
                                                    knowledge_function_1D_uniform,
                                                    param_server)

    scenario_risk_function = prior_knowledge_function.CalculateScenarioRiskFunction(
                              scenario_risk_templ_func_1D)

    indef_int = lambda x : 2*x**2 + 3*x
    uniform_prob = 1.0/10.0
    desired_normalization_const = 1.0/(uniform_prob*(indef_int(7.0)- indef_int(-3.0)))
    self.assertAlmostEqual(scenario_risk_function.normalization_constant,
              desired_normalization_const, 1000)







if __name__ == '__main__':
  unittest.main()