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
from bark.core.models.behavior import *

import matplotlib

import matplotlib.pyplot as plt
matplotlib.use("Qt5Agg")


import scipy

class PyBehaviorSpaceTests(unittest.TestCase):
  def test_default_config_sampling(self):
    param_server = ParameterServer()
    space = BehaviorSpace(param_server)
    sampled_parameters, model_type, \
    prob_prior, prob_sampling = space.sample_behavior_parameters()
    print(model_type)
    behavior = eval("{}(sampled_parameters)".format(model_type))
    print(sampled_parameters.ConvertToDict())
    param_server.Save("behavior_space_defaults_sampling.json")

    params_loaded = ParameterServer(filename="behavior_space_defaults_sampling.json")

    params_loaded["BehaviorSpace"]["Definition"]["SpaceBoundaries"]["BehaviorIDMStochastic"]["SpacingDistribution"] = [1.23, 20.2]
    params_loaded["BehaviorSpace"]["Sampling"]["BehaviorIDMStochastic"]["SpacingDistribution"]["StdRange"] = [0.2, 0.4]
    params_loaded["BehaviorSpace"]["Sampling"]["BehaviorIDMStochastic"]["SpacingDistribution"]["DistributionType"] = "NormalDistribution1D"

    params_loaded["BehaviorSpace"]["Definition"]["SpaceBoundaries"]["BehaviorIDMStochastic"]["DesiredVelDistribution"] = [4.5, 6.07]
    params_loaded["BehaviorSpace"]["Sampling"]["BehaviorIDMStochastic"]["DesiredVelDistribution"]["Width"] = [0.1, 0.3]
    params_loaded["BehaviorSpace"]["Sampling"]["BehaviorIDMStochastic"]["DesiredVelDistribution"]["DistributionType"] = "UniformDistribution1D"

    params_loaded["BehaviorSpace"]["Definition"]["SpaceBoundaries"]["BehaviorIDMStochastic"]["MaxAccDistribution"] = [1.7]
    params_loaded["BehaviorSpace"]["Sampling"]["BehaviorIDMStochastic"]["MaxAccDistribution"]["DistributionType"] = "FixedValue"

    params_loaded["BehaviorSpace"]["Definition"]["SpaceBoundaries"]["BehaviorIDMStochastic"]["ComftBrakingDistribution"] = [20]
    params_loaded["BehaviorSpace"]["Sampling"]["BehaviorIDMStochastic"]["ComftBrakingDistribution"]["DistributionType"] = "FixedValue"
    space = BehaviorSpace(params_loaded) 

    num_sampled_parameters = 100
    for _ in range(0, num_sampled_parameters):
      sampled_parameters, model_type, \
      prob_prior, prob_sampling = space.sample_behavior_parameters()
      behavior = eval("{}(sampled_parameters)".format(model_type))

      self.assertEqual(sampled_parameters["BehaviorIDMStochastic"]["MaxAccDistribution"]["DistributionType", "", ""], "FixedValue")
      self.assertEqual(sampled_parameters["BehaviorIDMStochastic"]["MaxAccDistribution"]["FixedValue", "", 1.0], [1.7]) 

      self.assertEqual(sampled_parameters["BehaviorIDMStochastic"]["ComftBrakingDistribution"]["DistributionType", "", ""], "FixedValue")
      self.assertEqual(sampled_parameters["BehaviorIDMStochastic"]["ComftBrakingDistribution"]["FixedValue", "", 1.2], [20])
      
      self.assertEqual(sampled_parameters["BehaviorIDMStochastic"]["SpacingDistribution"]["DistributionType", "", ""], "NormalDistribution1D")
      self.assertTrue(sampled_parameters["BehaviorIDMStochastic"]["SpacingDistribution"]["StdDev", "", 0.2] <= 0.4)
      self.assertTrue(sampled_parameters["BehaviorIDMStochastic"]["SpacingDistribution"]["StdDev", "", 0.1] >= 0.2)

      self.assertEqual(sampled_parameters["BehaviorIDMStochastic"]["DesiredVelDistribution"]["DistributionType", "", ""], "UniformDistribution1D")
      self.assertTrue(sampled_parameters["BehaviorIDMStochastic"]["DesiredVelDistribution"]["LowerBound", "", 1.0]>= 4.5)
      lb = sampled_parameters["BehaviorIDMStochastic"]["DesiredVelDistribution"]["LowerBound", "", 1.0]
      self.assertTrue(sampled_parameters["BehaviorIDMStochastic"]["DesiredVelDistribution"]["UpperBound", "", 0.2] >= lb+0.1)
      self.assertTrue(sampled_parameters["BehaviorIDMStochastic"]["DesiredVelDistribution"]["UpperBound", "", 0.1] <= lb+0.3)

  def test_partition_sampling(self):
    param_server = ParameterServer()
    space = BehaviorSpace(param_server)
    sampled_parameters, model_type, \
            prob_prior, prob_sampling = space.sample_behavior_parameters()
    print(model_type)
    behavior = eval("{}(sampled_parameters)".format(model_type))
    print(sampled_parameters.ConvertToDict())
    param_server.Save("behavior_space_partition_sampling.json")

    params_loaded = ParameterServer(filename="behavior_space_partition_sampling.json")

    params_loaded["BehaviorSpace"]["Definition"]["SpaceBoundaries"]["BehaviorIDMStochastic"]["SpacingDistribution"] = [2.5, 3.5]
    params_loaded["BehaviorSpace"]["Sampling"]["BehaviorIDMStochastic"]["SpacingDistribution"]["StdRange"] = [0.2, 0.4]
    params_loaded["BehaviorSpace"]["Sampling"]["BehaviorIDMStochastic"]["SpacingDistribution"]["DistributionType"] = "NormalDistribution1D"
    params_loaded["BehaviorSpace"]["Sampling"]["BehaviorIDMStochastic"]["SpacingDistribution"]["Partitions"] = 4
    params_loaded["BehaviorSpace"]["Sampling"]["BehaviorIDMStochastic"]["SpacingDistribution"]["SelectedPartition"] = 2

    params_loaded["BehaviorSpace"]["Definition"]["SpaceBoundaries"]["BehaviorIDMStochastic"]["DesiredVelDistribution"] = [4.8, 6.0]
    params_loaded["BehaviorSpace"]["Sampling"]["BehaviorIDMStochastic"]["DesiredVelDistribution"]["Width"] = [0.05, 0.1]
    params_loaded["BehaviorSpace"]["Sampling"]["BehaviorIDMStochastic"]["DesiredVelDistribution"]["DistributionType"] = "UniformDistribution1D"
    params_loaded["BehaviorSpace"]["Sampling"]["BehaviorIDMStochastic"]["DesiredVelDistribution"]["Partitions"] =  12
    params_loaded["BehaviorSpace"]["Sampling"]["BehaviorIDMStochastic"]["DesiredVelDistribution"]["SelectedPartition"] = 7

    params_loaded["BehaviorSpace"]["Definition"]["SpaceBoundaries"]["BehaviorIDMStochastic"]["MaxAccDistribution"] = [1.7]
    params_loaded["BehaviorSpace"]["Sampling"]["BehaviorIDMStochastic"]["MaxAccDistribution"]["DistributionType"] = "FixedValue"

    params_loaded["BehaviorSpace"]["Definition"]["SpaceBoundaries"]["BehaviorIDMStochastic"]["ComftBrakingDistribution"] = [20]
    params_loaded["BehaviorSpace"]["Sampling"]["BehaviorIDMStochastic"]["ComftBrakingDistribution"]["DistributionType"] = "FixedValue"
    space = BehaviorSpace(params_loaded) 

    num_sampled_parameters = 100
    for _ in range(0, num_sampled_parameters):
      sampled_parameters, model_type, \
            prob_prior, prob_sampling = space.sample_behavior_parameters()
      behavior = eval("{}(sampled_parameters)".format(model_type))

      self.assertEqual(sampled_parameters["BehaviorIDMStochastic"]["MaxAccDistribution"]["DistributionType", "", ""], "FixedValue")
      self.assertEqual(sampled_parameters["BehaviorIDMStochastic"]["MaxAccDistribution"]["FixedValue", "", 1.0], [1.7]) 

      self.assertEqual(sampled_parameters["BehaviorIDMStochastic"]["ComftBrakingDistribution"]["DistributionType", "", ""], "FixedValue")
      self.assertEqual(sampled_parameters["BehaviorIDMStochastic"]["ComftBrakingDistribution"]["FixedValue", "", 1.2], [20])
      
      self.assertEqual(sampled_parameters["BehaviorIDMStochastic"]["SpacingDistribution"]["DistributionType", "", ""], "NormalDistribution1D")
      self.assertTrue(sampled_parameters["BehaviorIDMStochastic"]["SpacingDistribution"]["StdDev", "", 0.5] <= 0.4)
      self.assertTrue(sampled_parameters["BehaviorIDMStochastic"]["SpacingDistribution"]["StdDev", "", 0.1] >= 0.2)
      self.assertTrue(sampled_parameters["BehaviorIDMStochastic"]["SpacingDistribution"]["Mean", "", 0.1] >= 3.0)
      self.assertTrue(sampled_parameters["BehaviorIDMStochastic"]["SpacingDistribution"]["Mean", "", 5.0] <= 3.25)

      self.assertEqual(sampled_parameters["BehaviorIDMStochastic"]["DesiredVelDistribution"]["DistributionType", "", ""], "UniformDistribution1D")
      self.assertTrue(sampled_parameters["BehaviorIDMStochastic"]["DesiredVelDistribution"]["LowerBound", "", 1.0]>= 5.5)
      self.assertTrue(sampled_parameters["BehaviorIDMStochastic"]["DesiredVelDistribution"]["UpperBound", "", 1.0]<= 5.6)
      lb = sampled_parameters["BehaviorIDMStochastic"]["DesiredVelDistribution"]["LowerBound", "", 1.0]
      ub = sampled_parameters["BehaviorIDMStochastic"]["DesiredVelDistribution"]["UpperBound", "", 1.0]
      self.assertTrue( ub - lb >= 0.05)
      self.assertTrue( ub - lb <= 0.1)

  def test_default_config_hypothesis_creation(self):
    param_server = ParameterServer()
    space = BehaviorSpace(param_server)
    hypothesis_set, hypothesis_parameters = space.create_hypothesis_set()
    num_hypothesis_desired = param_server["BehaviorSpace"]["Hypothesis"]["Partitions"]["BehaviorIDMStochastic"]["HeadwayDistribution"]
    self.assertEqual(len(hypothesis_set), num_hypothesis_desired)

    default_range = [3.0, 4.0]
    for idx, hypothesis in enumerate(hypothesis_set):
      params = hypothesis.params
      self.assertAlmostEquals(params.getReal("BehaviorIDMStochastic::HeadwayDistribution::LowerBound", "", 0.0), \
                 default_range[0] + idx*1/num_hypothesis_desired, 5)
      self.assertAlmostEquals(params.getReal("BehaviorIDMStochastic::HeadwayDistribution::UpperBound", "", 0.0),\
                 default_range[0] + (idx+1)*1/num_hypothesis_desired, 5)

    param_server.Save("behavior_space_defaults_hypothesis.json")
    params_loaded = ParameterServer(filename="behavior_space_defaults_hypothesis.json")
    space = BehaviorSpace(params_loaded)
    _,_ = space.create_hypothesis_set()

  def test_cover_hypothesis_creation(self):
    param_server = ParameterServer(log_if_default=False)
    behavior_space_range1 = param_server["BehaviorSpace"]["Definition"]["SpaceBoundaries"]["BehaviorIDMStochastic"]["HeadwayDistribution"] = [5.34, 10.0]
    behavior_space_range2 = param_server["BehaviorSpace"]["Definition"]["SpaceBoundaries"]["BehaviorIDMStochastic"]["SpacingDistribution"] = [1.3434, 10.0]
    behavior_space_fixed_val = param_server["BehaviorSpace"]["Definition"]["SpaceBoundaries"]["BehaviorIDMStochastic"]["DesiredVelDistribution"] = [3.4545]
    space = BehaviorSpace(param_server)
    hypothesis_set, hypothesis_parameters = space.create_cover_hypothesis()
    num_hypothesis_desired = param_server["BehaviorSpace"]["Hypothesis"]["Partitions"]["BehaviorIDMStochastic"]["HeadwayDistribution"]
    self.assertEqual(len(hypothesis_set), 1)

    params = hypothesis_set[0].params
    self.assertAlmostEquals(params.getReal("BehaviorIDMStochastic::HeadwayDistribution::LowerBound", "", 0.0), \
                behavior_space_range1[0], 5)
    self.assertAlmostEquals(params.getReal("BehaviorIDMStochastic::HeadwayDistribution::UpperBound", "", 0.0),\
                 behavior_space_range1[1], 5)
    self.assertAlmostEquals(params.getReal("BehaviorIDMStochastic::SpacingDistribution::LowerBound", "", 0.0), \
                behavior_space_range2[0], 5)
    self.assertAlmostEquals(params.getReal("BehaviorIDMStochastic::SpacingDistribution::UpperBound", "", 0.0),\
                 behavior_space_range2[1], 5)
    self.assertAlmostEquals(params.getListFloat("BehaviorIDMStochastic::DesiredVelDistribution::FixedValue", "", [0.0])[0],\
                 behavior_space_fixed_val[0], 5)

  def test_multiple_hypothesis_sets_creation(self):
    param_server = ParameterServer()
    behavior_space_range1 = param_server["BehaviorSpace"]["Definition"]["SpaceBoundaries"]["BehaviorIDMStochastic"]["HeadwayDistribution"] = [5.3434, 10.14]
    behavior_space_range2 = param_server["BehaviorSpace"]["Definition"]["SpaceBoundaries"]["BehaviorIDMStochastic"]["ComftBrakingDistribution"] = [1.0]
    behavior_space_range3 = param_server["BehaviorSpace"]["Definition"]["SpaceBoundaries"]["BehaviorIDMStochastic"]["MaxAccDistribution"] = [15.3]
    behavior_space_range4 = param_server["BehaviorSpace"]["Definition"]["SpaceBoundaries"]["BehaviorIDMStochastic"]["SpacingDistribution"] = [1.0, 2.0]
    behavior_space_range5 = param_server["BehaviorSpace"]["Definition"]["SpaceBoundaries"]["BehaviorIDMStochastic"]["DesiredVelDistribution"] = [3, 4.5]
    behavior_space_range6 = param_server["BehaviorSpace"]["Definition"]["SpaceBoundaries"]["BehaviorIDMStochastic"]["CoolnessFactorDistribution"] = [ 0.99 ]
    space = BehaviorSpace(param_server)
    desired_splits = [2, 8]
    hypothesis_set_collection = space.create_multiple_hypothesis_sets(splits=[2, 8])

    self.assertEqual(len(hypothesis_set_collection), len(desired_splits))

    bounds_collected_split = {}
    for split, (hypothesis_set, params_sets) in hypothesis_set_collection.items():
      num_hypothesis_desired = split
      self.assertEqual(len(hypothesis_set), split**3)
      bounds_collected_split[split] = defaultdict(list)
      bounds_collected = bounds_collected_split[split]
      for idx, hypothesis in enumerate(hypothesis_set):
        params = hypothesis.params
        lower = params.getReal("BehaviorIDMStochastic::HeadwayDistribution::LowerBound", "", 0.0)
        upper = params.getReal("BehaviorIDMStochastic::HeadwayDistribution::UpperBound", "", 0.0)
        bounds_collected[1].append([lower, upper])

        self.assertAlmostEquals(params.getListFloat("BehaviorIDMStochastic::ComftBrakingDistribution::FixedValue", "", [4345.0])[0], \
                  behavior_space_range2[0], 5)

        self.assertAlmostEquals(params.getListFloat("BehaviorIDMStochastic::MaxAccDistribution::FixedValue", "", [2334.0])[0], \
                  behavior_space_range3[0], 5)

        self.assertAlmostEquals(params.getListFloat("BehaviorIDMStochastic::CoolnessFactorDistribution::FixedValue", "", [2334.0])[0], \
                  behavior_space_range6[0], 5)

        lower = params.getReal("BehaviorIDMStochastic::SpacingDistribution::LowerBound", "", 0.0)
        upper = params.getReal("BehaviorIDMStochastic::SpacingDistribution::UpperBound", "", 0.0)
        bounds_collected[4].append([lower, upper])

        lower = params.getReal("BehaviorIDMStochastic::DesiredVelDistribution::LowerBound", "", 0.0)
        upper = params.getReal("BehaviorIDMStochastic::DesiredVelDistribution::UpperBound", "", 0.0)
        bounds_collected[5].append([lower, upper])

    desired_ranges = {1: behavior_space_range1, 4 : behavior_space_range4, 5 : behavior_space_range5}
    for split in desired_splits: 
      bounds_collected = bounds_collected_split[split]
      num_hypothesis_desired = split
      for behavior_range, bounds in bounds_collected.items():
          tmp = set(tuple(bound) for bound in bounds)
          splits = sorted(list(tmp), key=itemgetter(0))
          for idx, split in enumerate(splits):
            desired_range = desired_ranges[behavior_range]
            self.assertAlmostEquals(split[0], \
                      desired_range[0] + idx*(desired_range[1]-desired_range[0])/num_hypothesis_desired, 2)
            self.assertAlmostEquals(split[1],\
                      desired_range[0] + (idx+1)*(desired_range[1]-desired_range[0])/num_hypothesis_desired, 2)

  def test_prior_knowledge_probability_estimation(self):
    params_server = ParameterServer()
    # fixed param ranges without sampling for all distributions ...
    params_server["BehaviorSpace"]["Sampling"]["BehaviorIDMStochastic"]["SpacingDistribution"]["DistributionType"] = "FixedValue"
    params_server["BehaviorSpace"]["Definition"]["SpaceBoundaries"]["BehaviorIDMStochastic"]["SpacingDistribution"] = [2]
    params_server["BehaviorSpace"]["Sampling"]["BehaviorIDMStochastic"]["MaxAccDistribution"]["DistributionType"] = "FixedValue"
    params_server["BehaviorSpace"]["Definition"]["SpaceBoundaries"]["BehaviorIDMStochastic"]["MaxAccDistribution"] = [3]
    params_server["BehaviorSpace"]["Sampling"]["BehaviorIDMStochastic"]["ComftBrakingDistribution"]["DistributionType"] = "FixedValue"
    params_server["BehaviorSpace"]["Definition"]["SpaceBoundaries"]["BehaviorIDMStochastic"]["ComftBrakingDistribution"] = [4]
    params_server["BehaviorSpace"]["Sampling"]["BehaviorIDMStochastic"]["CoolnessFactorDistribution"]["DistributionType"] = "FixedValue"
    params_server["BehaviorSpace"]["Definition"]["SpaceBoundaries"]["BehaviorIDMStochastic"]["CoolnessFactorDistribution"] = [2.5]
    params_server["BehaviorSpace"]["Sampling"]["BehaviorIDMStochastic"]["HeadwayDistribution"]["DistributionType"] = "FixedValue"
    params_server["BehaviorSpace"]["Definition"]["SpaceBoundaries"]["BehaviorIDMStochastic"]["HeadwayDistribution"] = [1.2]

    # .. except one param distribution
    params_server["BehaviorSpace"]["Definition"]["SpaceBoundaries"]["BehaviorIDMStochastic"]["DesiredVelDistribution"] = [0.8, 10.8]
    params_server["BehaviorSpace"]["Sampling"]["BehaviorIDMStochastic"]["DesiredVelDistribution"]["Width"] = [0.1, 0.3]
    params_server["BehaviorSpace"]["Sampling"]["BehaviorIDMStochastic"]["DesiredVelDistribution"]["DistributionType"] = "FixedValue"
    params_server["BehaviorSpace"]["Sampling"]["BehaviorIDMStochastic"]["DesiredVelDistribution"]["Partitions"] = 10
    params_server["BehaviorSpace"]["Sampling"]["BehaviorIDMStochastic"]["DesiredVelDistribution"]["SelectedPartition"] = 1

    # parameterize prior knowledge function
    truncated_region = [1.8, 2.8] # equal to desired vel distribution boundaries, selected partition 1
    mean = 4
    std = 0.1
    params_server["BehaviorSpace"]["Definition"]["PriorKnowledgeFunction"]["TruncatedNormalKnowledgeFunctionDefinition"]\
        ["BehaviorIDMStochastic::DesiredVelDistribution"]["Mean"] = mean
    params_server["BehaviorSpace"]["Definition"]["PriorKnowledgeFunction"]["TruncatedNormalKnowledgeFunctionDefinition"]\
        ["BehaviorIDMStochastic::DesiredVelDistribution"]["Std"] = std
    a, b = (truncated_region[0] - mean) / std, (truncated_region[1] - mean) / std
    dist_func = scipy.stats.truncnorm(a=a, b=b, loc=mean, scale=std)

    space = BehaviorSpace(params_server) 
    num_sampled_parameters = 100
    for _ in range(0, num_sampled_parameters):
      sampled_parameters, model_type, \
            prob_prior, prob_sampling = space.sample_behavior_parameters()
      sampled_value = sampled_parameters["BehaviorIDMStochastic"]["DesiredVelDistribution"]["FixedValue", "", 1000.0]
      # calculate probability of fixed value under truncated normal with given parameters
      desired_prob_prior = dist_func.pdf(sampled_value)
      self.assertAlmostEqual(prob_prior, desired_prob_prior, 4)
      self.assertAlmostEqual(prob_sampling, 1.0, 4) # probability of uniform distribution density between 1.8 and 2.8

    # plot prior knowledge
    matplotlib.use("Qt5Agg")
    prior_knowledge_function_def = space.get_prior_knowledge_function().knowledge_function_definition
    dist = prior_knowledge_function_def.GetDistFuncOfRegion("BehaviorIDMStochastic::DesiredVelDistribution")
    fig, ax = plt.subplots(1, 1)
    x = np.linspace(-5.0, 5.0, 1000)
    ax.plot(x, dist.pdf(x), 'k-', lw=2)
    fig.show()

    # calculate scenario risk function
  #  risk_function = space.get_scenario_risk_function()



if __name__ == '__main__':
  unittest.main()