# Copyright (c) 2020 fortiss GmbH
#
# Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
# Tobias Kessler
#
# This software is released under the MIT License.
# https://opensource.org/licenses/MIT

try:
  debugging=True 
  import debug_settings
except:
  print("No debugging")
  debugging=False

import unittest
import os
from bark.runtime.scenario.scenario_generation.scenario_generation\
  import ScenarioGeneration

from bark.runtime.scenario.scenario import SetMapfileDirectory, GetMapfileDirectory

from bark.runtime.scenario.scenario_generation.configurable_scenario_generation \
  import ConfigurableScenarioGeneration, add_config_reader_module
from bark.runtime.commons import ParameterServer

from bark_mcts.runtime.scenario.scenario_set_generation import BehaviorHypothesisScenarioSetGeneration


import os

if debugging:
  SetMapfileDirectory("bazel-bin/bark_mcts/runtime/tests/py_scenario_set_generation_tests.runfiles/bark_project")


class ScenarioGenerationTests(unittest.TestCase):
  def test_hypothesis_scenario_set_generation(self):
    sink_source_dict = {
      "SourceSink": [[5111.626, 5006.8305],  [5110.789, 5193.1725] ],
      "Description": "left_lane",
      "ConfigAgentStatesGeometries": {"Type": "UniformVehicleDistribution", "LanePositions": [0]},
      "ConfigBehaviorModels": {"Type": "BehaviorSpaceSampling", "ModelType" : "BehaviorIDMStochastic", \
           "ModelParams" : {"BehaviorIDMStochastic::HeadwayDistribution::Range" : [10, 20],
                            "BehaviorIDMStochastic::HeadwayDistribution::Width": [0.5, 1.0]}},
      "ConfigExecutionModels": {"Type": "FixedExecutionType"},
      "ConfigDynamicModels": {"Type": "FixedDynamicType"},
      "ConfigGoalDefinitions": {"Type": "FixedGoalTypes"},
      "ConfigControlledAgents": {"Type": "NoneControlled"},
      "AgentParams" : {}
    }

    params = ParameterServer()
    params["Scenario"]["Generation"]["ConfigurableScenarioGeneration"]["SinksSources"] = [sink_source_dict]
    add_config_reader_module("bark_mcts.runtime.scenario.behavior_space_sampling")
    _ = ConfigurableScenarioGeneration(num_scenarios=2,params=params)
    params.Save(filename="test.json")
    set_generation = BehaviorHypothesisScenarioSetGeneration(params,
                                               partition_specs= {
                                                "BehaviorIDMStochastic::HeadwayDistribution" : 10,
                                                "BehaviorIDMStochastic::SpacingDistribution": 2,
                                                "BehaviorIDMStochastic::MaxAccDistribution": None,
                                                "BehaviorIDMStochastic::DesiredVelDistribution": None,
                                                "BehaviorIDMStochastic::ComftBrakingDistribution": None,
                                                "BehaviorIDMStochastic::CoolnessFactorDistribution": 2
                                              },
                                              behavior_space_root_name = "Scenario::Generation::ConfigurableScenarioGeneration::SinksSources")
    scenario_sets_dict = set_generation.GenerateSets("hypothesis_sets")
    self.assertEqual(len(scenario_sets_dict), 40)

if __name__ == '__main__':
  unittest.main()
    