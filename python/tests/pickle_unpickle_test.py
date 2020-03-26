import unittest
import pickle

# note: run this test with bazel test //python:pickle_unpickle_test --define planner_uct=true
# it uses the bark python module


from bark.models.behavior import *
from bark.models.dynamic import SingleTrackModel
from modules.runtime.commons.parameters import ParameterServer

def pickle_unpickle(object):
    with open('temp.pickle','wb') as f:
        pickle.dump(object,f)
    object = None
    with open( 'temp.pickle', "rb" ) as f:
        object = pickle.load(f)
    return object


class PickleTests(unittest.TestCase):
    def test_behavior_uct_single_agent_pickle(self):
        params = ParameterServer()
        behavior = BehaviorUCTSingleAgent(params)
        unpickled = pickle_unpickle(behavior)

    def test_behavior_uct_single_agent_macro_actions_pickle(self):
        params = ParameterServer()
        behavior = BehaviorUCTSingleAgentMacroActions(params)
        unpickled = pickle_unpickle(behavior)

    def test_behavior_uct_single_agent_macro_actions_pickle(self):
        params = ParameterServer()
        dyn_model = SingleTrackModel(params)
        ego_behavior = BehaviorMPMacroActions(dyn_model, params)
        hypothesis = BehaviorHypothesisIDMStochasticHeadway(params)
        hypothesis_list = []
        hypothesis_list.append(hypothesis)
        behavior = BehaviorUCTHypothesis(params, ego_behavior, hypothesis_list)
        unpickled = pickle_unpickle(behavior)



if __name__ == '__main__':
    unittest.main()