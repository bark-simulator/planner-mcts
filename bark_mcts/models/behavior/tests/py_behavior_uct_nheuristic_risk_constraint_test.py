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


import unittest
import numpy as np
import time
import logging
import gym
from bark.runtime.commons import ParameterServer
from bark.runtime.viewer import MPViewer
from bark.runtime.commons import XodrParser
from bark.core.models.execution import ExecutionModelInterpolate
from bark.core.models.dynamic import SingleTrackModel, StateDefinition
from bark.core.world import World, MakeTestWorldHighway
from bark.core.world.goal_definition import GoalDefinitionPolygon, GoalDefinitionStateLimitsFrenet
from bark.core.world.agent import Agent
from bark.core.world.map import MapInterface, Roadgraph
from bark.core.geometry.standard_shapes import CarLimousine, CarRectangle
from bark.core.geometry import Point2d, Polygon2d, Line2d
from bark.core.world.evaluation import EvaluatorDrivableArea
from bark.core.world.opendrive import OpenDriveMap, XodrRoad, PlanView, \
    MakeXodrMapOneRoadTwoLanes, XodrLaneSection, XodrLane
from bark.runtime.viewer.video_renderer import VideoRenderer
from bark.core.models.behavior import *
import os
import bark_ml.environments.gym
logging.basicConfig()
logging.getLogger().setLevel(logging.INFO)
logging.info("Running on process with ID: {}".format(os.getpid()))

from bark.world.tests.python_behavior_model import PythonDistanceBehavior

from bark_ml.core.observers import NearestObserver
from bark_ml.library_wrappers.lib_fqf_iqn_qrdqn.agent import ImitationAgent
from bark_mcts.models.behavior.hypothesis.behavior_space.behavior_space import BehaviorSpace

def create_nheuristic_behavior(model_file_name):
    params = ParameterServer()
    # Model Definitions
    space = BehaviorSpace(params)
    hypothesis_set, hypothesis_parameters = space.create_hypothesis_set()
    observer = NearestObserver(params)
    behavior_model = BehaviorUCTNHeuristicRiskConstraint(params, hypothesis_set, None,\
                             model_file_name, observer)
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

num_actions = 5
class TestActionWrapper():
  @property
  def action_space(self):
    return gym.spaces.Discrete(num_actions)

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


def train_test_model():
    params = ParameterServer()
    env = gym.make("highway-v1", params=params)
    env._observer = NearestObserver(params)
    env._ml_behavior = TestActionWrapper()
    params["ML"]["BaseAgent"]["NumSteps"] = 2
    params["ML"]["BaseAgent"]["EvalInterval"] = 1000
    data = create_data(10000)
    demo_train = data[0:7000]
    demo_test = data[7001:]
    agent = ImitationAgent(agent_save_dir="./save_dir", demonstrations_train=demo_train,
                        demonstrations_test=demo_test,
                        env=env, params=params)
    agent.run()
    agent.save(checkpoint_type="last")
    return agent.get_script_filename(checkpoint_load="last")

class SystemTests(unittest.TestCase):
    """ This shall serve as a full system test, importing world, agent, and behavior models
    """
    #@unittest.skip
    def test_train_and_load_model_to_plan(self):
        model_file_name = train_test_model()
        nheuristic_behavior = create_nheuristic_behavior(model_file_name)
        world = create_world(nheuristic_behavior)
        
        params = ParameterServer()
        # viewer
        viewer = MPViewer(params=params, use_world_bounds=True)
        # World Simulation
        sim_step_time = params["simulation"]["step_time",
                                              "Step-time in simulation", 0.2]
        sim_real_time_factor = params["simulation"]["real_time_factor",
                                                    "execution in real-time or faster", 1]
        # Draw map
        video_renderer = VideoRenderer(renderer=viewer, world_step_time=sim_step_time)

        for _ in range(0, 5):
            world.Step(sim_step_time)
            viewer.clear()
            video_renderer.drawWorld(world)
            video_renderer.drawGoalDefinition(goal_definition)
            time.sleep(sim_step_time/sim_real_time_factor)

        video_renderer.export_video(filename="./test_video_intermediate", remove_image_dir=True)


if __name__ == '__main__':
    unittest.main()
