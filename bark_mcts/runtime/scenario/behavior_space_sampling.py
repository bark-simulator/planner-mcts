# Copyright (c) 2020 Julian Bernhard

# This software is released under the MIT License.
# https://opensource.org/licenses/MIT

import numpy as np

from bark.runtime.scenario.scenario_generation.config_readers.config_readers_interfaces import ConfigReaderBehaviorModels
from bark_mcts.models.behavior.hypothesis.behavior_space.behavior_space import BehaviorSpace

from bark.core.models.behavior import *
from bark.runtime.commons.parameters import ParameterServer


class BehaviorSpaceSampling(ConfigReaderBehaviorModels):
  def __init__(self, *args, **kwargs):
    super().__init__(*args, **kwargs)
    self.param_servers = []

  def create_from_config(self, config_param_object, road_corridor, agent_states,  **kwargs):
    behavior_space = BehaviorSpace(config_param_object)
    # now do the true sampling
    behavior_models = []
    behavior_model_types = []

    for _ in agent_states:
      model_params_sampled, model_type = behavior_space.sample_behavior_parameters(self.random_state)
      self.param_servers.append(model_params_sampled)
      bark_model, _ = self.model_from_model_type(model_type, model_params_sampled)
      behavior_models.append(bark_model)
      behavior_model_types.append(model_type)
    return behavior_models, {"behavior_model_types" : behavior_model_types}, config_param_object

  def model_from_model_type(self, model_type, params):
    bark_model = eval("{}(params)".format(model_type))
    return bark_model, params

  def get_param_servers(self):
    return self.param_servers