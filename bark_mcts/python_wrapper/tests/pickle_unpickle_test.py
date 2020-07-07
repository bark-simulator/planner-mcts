import unittest
import pickle

# note: run this test with bazel test //python:pickle_unpickle_test --define planner_uct=true
# it uses the bark python module


from bark.core.models.behavior import *
from bark.core.models.dynamic import SingleTrackModel
from bark.runtime.commons.parameters import ParameterServer

def pickle_unpickle(object):
    with open('temp.pickle','wb') as f:
        pickle.dump(object,f)
    object = None
    with open( 'temp.pickle', "rb" ) as f:
        object = pickle.load(f)
    return object


class PickleTests(unittest.TestCase):
    def test_behavior_hypothesis_uct_pickle(self):
        params = ParameterServer()
        params["BehaviorIDMStochastic"]["HeadwayDistribution"]["LowerBound"] = 1.343412233123232323
        params["BehaviorIDMStochastic"]["HeadwayDistribution"]["UpperBound"] = 1.75656563123232323
        params["BehaviorHypothesisIDM"]["NumSamples"] = 13423434
        ego_behavior = BehaviorMPMacroActions(params)
        hypothesis = BehaviorHypothesisIDM(params)
        hypothesis_list = []
        hypothesis_list.append(hypothesis)
        behavior = BehaviorUCTHypothesis(params, [hypothesis])
        unpickled = pickle_unpickle(behavior)
        unpickled_hypothesis = unpickled.hypotheses
        self.assertEqual(len(unpickled_hypothesis), 1)
        hyp1 = unpickled_hypothesis[0]
        self.assertTrue(isinstance(hyp1, BehaviorHypothesisIDM))
        self.assertAlmostEqual(hyp1.params.getReal("BehaviorIDMStochastic::HeadwayDistribution::LowerBound", "", 1.0), \
                         1.343412233123232323, 5)
        self.assertAlmostEqual(hyp1.params.getReal("BehaviorIDMStochastic::HeadwayDistribution::UpperBound", "", 3.0), \
                         1.75656563123232323, 5)
        self.assertEqual(hyp1.params.getInt("BehaviorHypothesisIDM::NumSamples", "", 1), 13423434)



if __name__ == '__main__':
    unittest.main()