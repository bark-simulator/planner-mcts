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
      region_range = partition.definition["1DDimensionName"]
      width = region_range[1] -region_range[0]
      self.assertAlmostEqual(width, desired_width, 4)
      total_range += width
    self.assertAlmostEqual(total_range, 5.0)
    self.assertAlmostEqual(partitions[0].definition["1DDimensionName"][0], -2.0)
    self.assertAlmostEqual(partitions[-1].definition["1DDimensionName"][1],  5.0)


  def test_1D_functions_uniform(self):
    param_server = ParameterServer()
    prior_knowledge_region = PriorKnowledgeRegion({"1DDimensionName" : (-3.0, 7.0)})

    class TestKnowledgeFunctionDef(ScenarioRiskFunctionDefinition):
      def __init__(self, supporting_region):
        super().__init__(supporting_region)
        self.supporting_region = supporting_region

      def CalculateIntegral(self, region_boundaries):
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

    knowledge_function_def = TestKnowledgeFunctionDef(prior_knowledge_region.definition)
    prior_knowledge_function = PriorKnowledgeFunction(prior_knowledge_region, 
                                                    knowledge_function_def,
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

    class TestKnowledgeFunctionDef(ScenarioRiskFunctionDefinition):
      def __init__(self, supporting_region):
        super().__init__(supporting_region)
        self.supporting_region = supporting_region

      def CalculateIntegral(self, region_boundaries):
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
      indef_int = lambda x : -1.0/2.0*x**2 + 10*x 
      if region_range[1] > 0 and region_range[0] < 0:
        return abs(indef_int(0) - indef_int(region_range[0])) + \
              abs(indef_int(region_range[1]) - indef_int(0))
      else:
        return abs(indef_int(region_range[1]) - indef_int(region_range[0]))

    knowledge_function_def = TestKnowledgeFunctionDef(prior_knowledge_region.definition)
    prior_knowledge_function = PriorKnowledgeFunction(prior_knowledge_region, 
                                                    knowledge_function_def,
                                                    param_server)

    scenario_risk_function = prior_knowledge_function.CalculateScenarioRiskFunction(
                              scenario_risk_templ_func_1D)

    indef_int = lambda x : -1.0/2.0*x**2 + 10*x 
    uniform_prob = 1.0/10.0
    area = abs(indef_int(0.0) - indef_int(-3.0)) + abs(indef_int(7.0)- indef_int(0.0))
    desired_normalization_const = 1.0/(uniform_prob*area)
    self.assertAlmostEqual(scenario_risk_function.normalization_constant,
              desired_normalization_const, 8)

  def test_1D_functions_complex(self):
    param_server = ParameterServer()
    param_server["PriorKnowledgeFunction"]["NumPartitionsIntegration"] = 1000
    prior_knowledge_region = PriorKnowledgeRegion({"1DDimensionName" : (-3.0, 7.0)})

    class TestKnowledgeFunctionDef(ScenarioRiskFunctionDefinition):
      def __init__(self, supporting_region):
        super().__init__(supporting_region)
        self.supporting_region = supporting_region

      def CalculateIntegral(self, region_boundaries):
        if not len(region_boundaries) == 1:
          raise ValueError("Only 1D knowledge function provided")
        # template function is a*x^2+ b*x + c then indefinite integral is a/3.0*x^3 + b*x^2/2.0 + c*x
        a1 = 10.0
        b1 = 3.0
        c1 = 2.0
        indef_int = lambda x : a1/3.0*x**3 + b1*x**2/2.0 + c1*x
        region_range = region_boundaries["1DDimensionName"]
        int0 = indef_int(region_range[0])
        int1 = indef_int(region_range[1])
        if (int0 < 0 and int1 > 0) or (int0 > 0 and int1 < 0):
          return abs(int0) + abs(int1)
        else:
          return abs(indef_int(region_range[1]) - indef_int(region_range[0]))

    def scenario_risk_templ_func_1D_linear(region_boundaries):
      if not len(region_boundaries) == 1:
        raise ValueError("Only 1D scenario risk function provided")
       # template function is a*x^2+ b*x + c then indefinite integral is a/3.0*x^3 + b*x^2/2.0 + c*x
      a2 = -0.5
      b2= 1.0
      indef_int = lambda x : a2/2.0*x**2 + b2*x
      region_range = region_boundaries["1DDimensionName"]
      int0 = indef_int(region_range[0])
      int1 = indef_int(region_range[1])
      if (int0 < 0 and int1 > 0) or (int0 > 0 and int1 < 0):
        return abs(int0) + abs(int1)
      else:
        return abs(indef_int(region_range[1]) - indef_int(region_range[0]))

    knowledge_function_def = TestKnowledgeFunctionDef(prior_knowledge_region.definition)
    prior_knowledge_function = PriorKnowledgeFunction(prior_knowledge_region, 
                                                    knowledge_function_def,
                                                    param_server)

    scenario_risk_function = prior_knowledge_function.CalculateScenarioRiskFunction(
                              scenario_risk_templ_func_1D_linear)

    a1 = 10.0
    b1 = 3.0
    c1 = 2.0
    a2 = -0.5
    b2= 1.0
    indef_int = lambda x : a1*a2*x**4/4 + a1*b2*x**3/3 + b1*a2*x**3/3 + b1*b2*x**2/2 + c1*a2*x**2/2 + c1*b2*x
    area = abs(indef_int(0.0) - indef_int(-3.0)) + abs(indef_int(7.0)- indef_int(0.0))
    desired_normalization_const = 1.0/(area)
    # only precision 5 working here!!!!
    self.assertAlmostEqual(scenario_risk_function.normalization_constant,
              desired_normalization_const, 5)

  def test_2D_functions_complex(self):
    param_server = ParameterServer()
    prior_knowledge_region = PriorKnowledgeRegion({"DimensionName1" : (1.0, 5.0), "DimensionName2" : (2.0, 8.0)})

    class TestKnowledgeFunctionDef(ScenarioRiskFunctionDefinition):
      def __init__(self, supporting_region):
        super().__init__(supporting_region)
        self.supporting_region = supporting_region

      def CalculateIntegral(self, region_boundaries):
        if not len(region_boundaries) == 2:
          raise ValueError("Only 2D knowledge function provided")
        # template function is a1*x*y then indefinite integral is a1*x''^2*y''^2/4 - a1*
        a1 = 10.0
        indef_int = lambda x1, x2, y1, y2: a1/4*x2**2*y2**2 - a1/4*x1**2*y2**2 - a1/4*x2**2*y1**2 + a1/4*x1**2*y1**2
        r1 = region_boundaries["DimensionName1"]
        r2 = region_boundaries["DimensionName2"]
        return abs(indef_int(r1[0], r1[1], r2[0], r2[1]))

    def scenario_risk_templ_func_2D_linear(region_boundaries):
      if not len(region_boundaries) == 2:
        raise ValueError("Only 2D scenario risk function provided")
       # template function is a*x^2+ b*x + c then indefinite integral is a/3.0*x^3 + b*x^2/2.0 + c*x
      a2 = -0.5
      indef_int = lambda x : a2/2.0*x**2 
      r1 = region_boundaries["DimensionName1"]
      r2 = region_boundaries["DimensionName2"]
      return abs(indef_int(r1[1]) - indef_int(r1[0]))*(r2[1] - r2[0])

    knowledge_function_def = TestKnowledgeFunctionDef(prior_knowledge_region.definition)
    prior_knowledge_function = PriorKnowledgeFunction(prior_knowledge_region, 
                                                    knowledge_function_def,
                                                    param_server)

    scenario_risk_function = prior_knowledge_function.CalculateScenarioRiskFunction(
                              scenario_risk_templ_func_2D_linear)

    a1 = 10.0
    a2 = -0.5
    indef_int = lambda x1, x2, y1, y2: a1*a2/6*x2**3*y2**2 - a1*a2/6*x1**3*y2**2 - a1*a2/6*x2**3*y1**2 + a1*a2/6*x1**3*y1**2
    area = indef_int(1.0, 5.0, 2.0, 8.0)
    desired_normalization_const = 1.0/(abs(area))
    self.assertAlmostEqual(scenario_risk_function.normalization_constant,
              desired_normalization_const, 8)


if __name__ == '__main__':
  unittest.main()