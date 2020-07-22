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
import itertools
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
    sink_source_dict = [{
        "SourceSink": [[5111.626, 5006.8305],  [5110.789, 5193.1725]],
        "Description": "left_lane",
        "ConfigAgentStatesGeometries": {"Type": "UniformVehicleDistribution", "LanePositions": [0]},
        "ConfigBehaviorModels": {"Type": "BehaviorSpaceSampling", "ModelType" : "BehaviorIDMStochastic"},
        "ConfigExecutionModels": {"Type": "FixedExecutionType"},
        "ConfigDynamicModels": {"Type": "FixedDynamicType"},
        "ConfigGoalDefinitions": {"Type": "FixedGoalTypes"},
        "ConfigControlledAgents": {"Type": "NoneControlled"},
        "AgentParams": {}
    },
    {
    "SourceSink": [[5111.626, 5006.8305],  [5110.789, 5193.1725]],
    "Description": "right_lane",
    "ConfigAgentStatesGeometries": {"Type": "UniformVehicleDistribution", "LanePositions": [1]},
    "ConfigBehaviorModels": {"Type": "BehaviorSpaceSampling", "ModelType" : "BehaviorIDMStochastic"},
    "ConfigExecutionModels": {"Type": "FixedExecutionType"},
    "ConfigDynamicModels": {"Type": "FixedDynamicType"},
    "ConfigGoalDefinitions": {"Type": "FixedGoalTypes"},
    "ConfigControlledAgents": {"Type": "RandomSingleAgent"},
    "AgentParams": {}
    }]

    params = ParameterServer()
    params["Scenario"]["Generation"]["ConfigurableScenarioGeneration"]["SinksSources"] = sink_source_dict
    add_config_reader_module("bark_mcts.runtime.scenario.behavior_space_sampling")
    _ = ConfigurableScenarioGeneration(num_scenarios=2,params=params)
    params.Save(filename="test.json")
    partition_specs = {
        "BehaviorIDMStochastic::HeadwayDistribution" : 10,
        "BehaviorIDMStochastic::SpacingDistribution": 2,
        "BehaviorIDMStochastic::MaxAccDistribution": None,
        "BehaviorIDMStochastic::DesiredVelDistribution": None,
        "BehaviorIDMStochastic::ComftBrakingDistribution": None,
        "BehaviorIDMStochastic::CoolnessFactorDistribution": 2
    }
    hierarchy_param_name = "Scenario::Generation::ConfigurableScenarioGeneration::SinksSources::ConfigBehaviorModels::BehaviorSpace::Sampling"
    set_generation = BehaviorHypothesisScenarioSetGeneration(params,
                                               partition_specs = partition_specs,
                                              hierarchy_param_name = hierarchy_param_name)
    scenario_sets_dict = set_generation.GetSets("hypothesis_sets")
    self.assertEqual(len(scenario_sets_dict), 40)


    combinations_selected_partitions = list(itertools.product(*[list(range(0, spec)) for spec in partition_specs.values() if spec]))
    for scenario_sets_name, scenario_set_params in scenario_sets_dict.items():
      scenario_set_params.Save(filename=scenario_sets_name+".json")
      for partition_name, partition_spec in partition_specs.items():
        sinks_sources = scenario_set_params.AddChild("Scenario::Generation::ConfigurableScenarioGeneration::SinksSources")
        for sink_source_param in sinks_sources:
          partition_spec_in_params = sink_source_param.AddChild("ConfigBehaviorModels::BehaviorSpace::Sampling::{}".\
                        format(partition_name))["Partitions"]
          self.assertEqual(partition_spec_in_params, partition_spec)

      for idx, comb in enumerate(combinations_selected_partitions):
        found = True
        for partition_name, partition_spec in partition_specs.items():
          idx_spec=0
          sinks_sources = scenario_set_params.AddChild("Scenario::Generation::ConfigurableScenarioGeneration::SinksSources")
          selected_partition_in_params = None
          for sink_source_param in sinks_sources:
            selected_partition_next_source_sink = sink_source_param.AddChild("ConfigBehaviorModels::BehaviorSpace::Sampling::{}".\
                          format(partition_name))["SelectedPartition"]
            if selected_partition_in_params:
              self.assertEqual(selected_partition_in_params, selected_partition_next_source_sink)
            selected_partition_in_params = selected_partition_next_source_sink
          found = found and comb[idx_spec] == selected_partition_in_params
          if not found:
            break
        if found:
          break
      del combinations_selected_partitions[idx]
    self.assertEqual(len(combinations_selected_partitions), 0)


if __name__ == '__main__':
  unittest.main()
    