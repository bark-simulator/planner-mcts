# Copyright (c) 2019 Julian Bernhard
#
# This software is released under the MIT License.
# https://opensource.org/licenses/MIT

try: 
  import debug_settings
except:
  print("No debugging")

import unittest

import bark.core

from bark.core.models.behavior import *
from bark.runtime.commons.parameters import ParameterServer
from bark.core.world import World, MakeTestWorldHighway, ObservedWorld

from bark_mcts.models.behavior.hypothesis.behavior_space.behavior_space import BehaviorSpace

class BeliefCalculatorTest(unittest.TestCase):
    def test_correct_types(self):
        params = ParameterServer()
        hypothesis1 = BehaviorHypothesisIDM(params)
        hypothesis2 = BehaviorHypothesisIDM(params)
        belief_calculator = BeliefCalculator(params, [hypothesis1, hypothesis2])
        params.Save(filename="./belief_calculator_default_params.json")
        world = MakeTestWorldHighway()
        agent_id_list = list(world.agents.keys())
        observed_world = ObservedWorld(world, agent_id_list[0])
        belief_calculator.BeliefUpdate(observed_world)
        beliefs = belief_calculator.GetBeliefs()
        self.assertEqual(len(beliefs), len(agent_id_list)-1)
        for agent_id, belief_list in beliefs.items():
          self.assertEqual(len(belief_list), 2)

    def test_belief_matching_behavior_space(self):
        params_server = ParameterServer()
        params_server["BehaviorSpace"]["Sampling"]["BehaviorIDMStochastic"]["SpacingDistribution"]["DistributionType"] = "FixedValue"
        params_server["BehaviorSpace"]["Definition"]["SpaceBoundaries"]["BehaviorIDMStochastic"]["SpacingDistribution"] = [2]
        params_server["BehaviorSpace"]["Sampling"]["BehaviorIDMStochastic"]["MaxAccDistribution"]["DistributionType"] = "FixedValue"
        params_server["BehaviorSpace"]["Definition"]["SpaceBoundaries"]["BehaviorIDMStochastic"]["MaxAccDistribution"] = [1.7]
        params_server["BehaviorSpace"]["Sampling"]["BehaviorIDMStochastic"]["ComftBrakingDistribution"]["DistributionType"] = "FixedValue"
        params_server["BehaviorSpace"]["Definition"]["SpaceBoundaries"]["BehaviorIDMStochastic"]["ComftBrakingDistribution"] = [4]
        params_server["BehaviorSpace"]["Sampling"]["BehaviorIDMStochastic"]["CoolnessFactorDistribution"]["DistributionType"] = "FixedValue"
        params_server["BehaviorSpace"]["Definition"]["SpaceBoundaries"]["BehaviorIDMStochastic"]["CoolnessFactorDistribution"] = [0.0]
        params_server["BehaviorSpace"]["Sampling"]["BehaviorIDMStochastic"]["HeadwayDistribution"]["DistributionType"] = "FixedValue"
        params_server["BehaviorSpace"]["Definition"]["SpaceBoundaries"]["BehaviorIDMStochastic"]["HeadwayDistribution"] = [1.2]
        params_server["BehaviorSpace"]["Definition"]["SpaceBoundaries"]["BehaviorIDMStochastic"]["DesiredVelDistribution"] = [5, 10]
        params_server["BehaviorSpace"]["Sampling"]["BehaviorIDMStochastic"]["DesiredVelDistribution"]["Width"] = [0.1, 0.3]
        params_server["BehaviorSpace"]["Sampling"]["BehaviorIDMStochastic"]["DesiredVelDistribution"]["DistributionType"] = "FixedValue"

        params_server_behavior = ParameterServer()
        behavior_space = BehaviorSpace(params_server_behavior)
        hypothesis_set, hypothesis_params = behavior_space.create_hypothesis_set_fixed_split(split = 2)
        world = MakeTestWorldHighway()
        for agent_id, agent in world.agents.items():
          agent.behavior_model = hypothesis_set[0]
        agent_id_list = list(world.agents.keys())
        observed_world = ObservedWorld(world, agent_id_list[0])
        belief_calculator = BeliefCalculator(params_server, hypothesis_set)
        for idx in range(0, 100):
          observed_world.Step(0.2)
          print(observed_world.agents)
          belief_calculator.BeliefUpdate(observed_world)
          beliefs = belief_calculator.GetBeliefs()
          print(beliefs)
        
        params_server_behavior.Save(filename="./default_params_behavior_space")

if __name__ == '__main__':
    unittest.main()
    print("test")