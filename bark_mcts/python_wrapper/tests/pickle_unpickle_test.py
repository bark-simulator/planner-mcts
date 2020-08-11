import unittest
import dill
import pickle

# note: run this test with bazel test //python:pickle_unpickle_test --define planner_uct=true
# it uses the bark python module


from bark.core.models.behavior import *
from bark.core.models.behavior.risk_calculation import *
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
    def test_behavior_uct_hypothesis_pickle(self):
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

    def test_behavior_uct_risk_constraintpickle(self):
        scenario_risk_func_before = lambda x: x
        params = ParameterServer()
        params["BehaviorIDMStochastic"]["HeadwayDistribution"]["LowerBound"] = 1.343412233123232323
        params["BehaviorIDMStochastic"]["HeadwayDistribution"]["UpperBound"] = 1.75656563123232323
        params["BehaviorHypothesisIDM"]["NumSamples"] = 13423434
        ego_behavior = BehaviorMPMacroActions(params)
        hypothesis = BehaviorHypothesisIDM(params)
        hypothesis_list = []
        hypothesis_list.append(hypothesis)
        scenario_risk_func_before = ScenarioRiskFunction(scenario_risk_func_before, 0.3456)
        behavior = BehaviorUCTRiskConstraint(params, [hypothesis], scenario_risk_func_before)
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

        scenario_risk_func = unpickled.scenario_risk_function
        self.assertEqual(scenario_risk_func, scenario_risk_func_before)



if __name__ == '__main__':
    unittest.main()