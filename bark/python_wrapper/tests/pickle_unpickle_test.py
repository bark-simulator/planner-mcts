import unittest
import pickle

# note: run this test with bazel test //python:pickle_unpickle_test --define planner_uct=true
# it uses the bark python module


from bark.models.behavior import *
from bark.models.dynamic import SingleTrackModel
from bark.runtime.commons.parameters import ParameterServer

def pickle_unpickle(object):
    with open('temp.pickle','wb') as f:
        pickle.dump(object,f)
    object = None
    with open( 'temp.pickle', "rb" ) as f:
        object = pickle.load(f)
    return object


class PickleTests(unittest.TestCase):
    def test_behavior_uct_single_agent_macro_actions_pickle(self):
        params = ParameterServer()
        behavior = BehaviorUCTSingleAgentMacroActions(params)
        unpickled = pickle_unpickle(behavior)

    def test_behavior_hypothesis_uct_pickle(self):
        params = ParameterServer()
        params["BehaviorIDMStochastic"]["HeadwayDistribution"]["LowerBound"] = 1.343412233123232323
        params["BehaviorIDMStochastic"]["HeadwayDistribution"]["UpperBound"] = 1.75656563123232323
        params["BehaviorHypothesisIDMStochastic"]["NumSamples"] = 13423434
        dyn_model = SingleTrackModel(params)
        ego_behavior = BehaviorMPMacroActions(dyn_model, params)
        hypothesis = BehaviorHypothesisIDMStochastic(params)
        hypothesis_list = []
        hypothesis_list.append(hypothesis)
        behavior = BehaviorUCTHypothesis(params, [hypothesis])
        unpickled = pickle_unpickle(behavior)
        unpickled_hypothesis = unpickled.hypotheses
        self.assertEqual(len(unpickled_hypothesis), 1)
        hyp1 = unpickled_hypothesis[0]
        self.assertTrue(isinstance(hyp1, BehaviorHypothesisIDMStochastic))
        self.assertAlmostEqual(hyp1.params.getReal("BehaviorIDMStochastic::HeadwayDistribution::LowerBound", "", 1.0), \
                         1.343412233123232323, 5)
        self.assertAlmostEqual(hyp1.params.getReal("BehaviorIDMStochastic::HeadwayDistribution::UpperBound", "", 3.0), \
                         1.75656563123232323, 5)
        self.assertEqual(hyp1.params.getInt("BehaviorHypothesisIDMStochastic::NumSamples", "", 1), 13423434)



if __name__ == '__main__':
    unittest.main()