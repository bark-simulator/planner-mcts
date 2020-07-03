# Copyright (c) 2020 fortiss GmbH
#
# Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
# Tobias Kessler
#
# This software is released under the MIT License.
# https://opensource.org/licenses/MIT

try: 
  import debug_settings
except:
  print("No debugging")

import unittest
import os
from bark.runtime.scenario.scenario_generation.scenario_generation\
  import ScenarioGeneration

from bark.runtime.scenario.scenario_generation.configurable_scenario_generation \
  import ConfigurableScenarioGeneration
from bark.runtime.commons import ParameterServer


import os


class ScenarioGenerationTests(unittest.TestCase):
  def test_configurable_scenario_generation_behavior_space_sampling(self):
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
    scenario_generation = ConfigurableScenarioGeneration(num_scenarios=2,params=params)
    scenario_generation.dump_scenario_list("test.scenario")

    scenario_loader = ScenarioGeneration()
    scenario_loader.load_scenario_list("test.scenario")

    self.assertEqual(len(scenario_loader._scenario_list), 2)
    self.assertEqual(len(scenario_loader._scenario_list[0]._agent_list), len(scenario_generation._scenario_list[0]._agent_list))

    scenario = scenario_loader.get_scenario(idx=0)

    params.Save("default_params_sampling.json")

    peristed_param_servers = scenario_generation.get_persisted_param_servers()
    behavior_models_params = peristed_param_servers["ConfigBehaviorModels"][0]["ParameterServers"]
    for behavior_param in behavior_models_params:
      dct = behavior_param.ConvertToDict()
      print(dct)

if __name__ == '__main__':
  unittest.main()
