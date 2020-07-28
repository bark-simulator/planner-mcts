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

class PyScenarioRiskFunctionTests(unittest.TestCase):
  def test_1D_risk_function_gauss(self):
    param_server = ParameterServer()



if __name__ == '__main__':
  unittest.main()