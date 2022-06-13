# Copyright (c) 2020 fortiss GmbH
#
# Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
# Tobias Kessler
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

try:
  import debug_settings
except:
  pass

import os
os.environ["TORCH_AUTOGRAD_SHUTDOWN_WAIT_LIMIT"] = "0"
import unittest
import numpy as np
import logging
import torch
import gym

from bark.runtime.commons import ParameterServer
from bark.runtime.viewer import MPViewer
from bark.core.models.execution import ExecutionModelInterpolate
from bark.core.models.dynamic import SingleTrackModel
from bark.core.world import World
from bark.core.world.goal_definition import GoalDefinitionStateLimitsFrenet
from bark.core.world.agent import Agent
from bark.core.world.map import MapInterface
from bark.core.geometry.standard_shapes import  CarRectangle
from bark.core.geometry import Point2d, Line2d
from bark.core.world.opendrive import    MakeXodrMapOneRoadTwoLanes
from bark.runtime.viewer.video_renderer import VideoRenderer
from bark.core.models.behavior import *
from bark_ml.library_wrappers.lib_fqf_iqn_qrdqn.tests.test_imitation_agent import TestActionWrapper, \
        TestObserver

logging.basicConfig()
logging.getLogger().setLevel(logging.INFO)
logging.info("Running on process with ID: {}".format(os.getpid()))

from bark_ml.core.observers import NearestObserver
from bark_ml.library_wrappers.lib_fqf_iqn_qrdqn.agent import ImitationAgent
from bark_mcts.models.behavior.hypothesis.behavior_space.behavior_space import BehaviorSpace

def create_nheuristic_behavior(model_file_name, nn_to_value_converter):
    params = ParameterServer(filename="bark_mcts/models/behavior/tests/data/nheuristic_test.json")
    # Model Definitions
    space = BehaviorSpace(params)
    hypothesis_set, hypothesis_parameters = space.create_hypothesis_set()
    observer = NearestObserver(params)
    behavior_model = BehaviorUCTNHeuristicRiskConstraint(params, hypothesis_set, None,\
                             model_file_name, observer, nn_to_value_converter)
    params.Save(filename="./mcts_nheuristic_default_params.json")
    return behavior_model


def create_world(nheuristic_behavior):
    params = ParameterServer()
    world = World(params)
    execution_model = ExecutionModelInterpolate(params)
    dynamic_model = SingleTrackModel(params)

    behavior_model2 = BehaviorConstantAcceleration(params)
    execution_model2 = ExecutionModelInterpolate(params)
    dynamic_model2 = SingleTrackModel(params)

    # Map Definition
    map_interface = MapInterface()
    xodr_map = MakeXodrMapOneRoadTwoLanes()
    map_interface.SetOpenDriveMap(xodr_map)
    world.SetMap(map_interface)

    # agent_2d_shape = CarLimousine()
    agent_2d_shape = CarRectangle()
    init_state = np.array([0, 3, -5.25, 0, 20])
    agent_params = params.AddChild("agent1")


    center_line = Line2d()
    center_line.AddPoint(Point2d(0.0, -1.75))
    center_line.AddPoint(Point2d(100.0, -1.75))

    max_lateral_dist = (0.4,0.5)
    max_orientation_diff = (0.08, 0.1)
    velocity_range = (5.0, 20.0)
    goal_definition = GoalDefinitionStateLimitsFrenet(center_line,
                    max_lateral_dist, max_orientation_diff,
                    velocity_range)

    agent = Agent(init_state, nheuristic_behavior, dynamic_model, execution_model,
                    agent_2d_shape, agent_params, goal_definition, map_interface)
    world.AddAgent(agent)

    init_state2 = np.array([0, 25, -5.25, 0, 0])
    agent2 = Agent(init_state2, behavior_model2, dynamic_model2, execution_model2,
                    agent_2d_shape, agent_params, goal_definition, map_interface)
    world.AddAgent(agent2)

    return world

num_actions = 4
class TestMotionPrimitiveBehavior:
  def __init__(self, num_actions):
    self._num_actions = num_actions

  def GetMotionPrimitives(self):
    return list(range(0,self._num_actions))

def action_values_at_state(state):
    envelope_costs = []
    collision_costs = []
    return_values = []
    for idx in range(1, num_actions+1):
        envelope_costs.append(state[0]* 1.0/idx + state[1]* 1.0/idx)
        collision_costs.append(state[1]* 1.0/idx*0.001 + state[3]* 1.0/idx*0.001)
        collision_costs.append(state[3]* 1.0/idx*0.2 + state[4]* 1.0/idx*0.1)
    action_values = []
    action_values.extend(envelope_costs)
    action_values.extend(collision_costs)
    action_values.extend(return_values)
    return [state, action_values]

def create_data(num):
    params = ParameterServer()
    observer = NearestObserver(params)
    observation_length = observer.observation_space.shape[0]
    observations = np.random.rand(num, observation_length)
    action_values_data = np.apply_along_axis(action_values_at_state, 1, observations)
    return action_values_data

class TestActionWrapper():
  @property
  def action_space(self):
    return gym.spaces.Discrete(num_actions)

class TestDemonstrationCollector:
  def __init__(self):
    self.data = create_data(1000)
    params = ParameterServer(filename="bark_mcts/models/behavior/tests/data/nheuristic_test.json")
    self._observer = NearestObserver(params)
    self._ml_behavior =  TestActionWrapper()
    self.motion_primitive_behavior = TestMotionPrimitiveBehavior(num_actions)

  def GetDemonstrationExperiences(self):
    return self.data

  @property
  def observer(self):
    return self._observer

  @property
  def ml_behavior(self):
    return self._ml_behavior

  def GetDirectory(self):
    return "./save_dir/collections"

def imitation_agent(layer_dims):
    params = ParameterServer()
    params["ML"]["BaseAgent"]["NumSteps"] = 2
    params["ML"]["BaseAgent"]["EvalInterval"] = 1
    params["ML"]["ImitationModel"]["EmbeddingDims"] = layer_dims
    agent = ImitationAgent(agent_save_dir="./save_dir", demonstration_collector=TestDemonstrationCollector(),
                          params=params)
    agent.run()
    agent.save(checkpoint_type="last")
    return agent.get_script_filename(checkpoint_load="last"), agent.nn_to_value_converter

class SystemTestNHeuristic(unittest.TestCase):
    # skip on ci since pytorch causes bazel test to hang
    # Uncomment to regenerate script model file
    @unittest.skip
    def test_system_test(self):
        model_file_name, nn_to_value_converter = imitation_agent([200, 100, 100])
        logging.info(f"Script File: {os.path.abspath(model_file_name)}")
        nheuristic_behavior = create_nheuristic_behavior(model_file_name, nn_to_value_converter)
        world = create_world(nheuristic_behavior)

        params = ParameterServer()

        # # World Simulation
        for _ in range(0, 5):
            world.Step(0.2)

if __name__ == '__main__':
  unittest.main()


