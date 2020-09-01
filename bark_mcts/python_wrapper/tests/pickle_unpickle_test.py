import unittest
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
        params = ParameterServer()
        params["BehaviorIDMStochastic"]["HeadwayDistribution"]["LowerBound"] = 1.343412233123232323
        params["BehaviorIDMStochastic"]["HeadwayDistribution"]["UpperBound"] = 1.75656563123232323
        params["BehaviorHypothesisIDM"]["NumSamples"] = 13423434
        ego_behavior = BehaviorMPMacroActions(params)
        hypothesis = BehaviorHypothesisIDM(params)
        hypothesis_list = []
        hypothesis_list.append(hypothesis)
        prior_knowledge_region_before = PriorKnowledgeRegion({"DimensionName1" : (1.0, 5.0), "DimensionName2" : (2.0, 8.0)})
        scenario_risk_params = ParameterServer()
        scenario_risk_params["LinearKnowledgeFunction"]["DimensionName1"]["a"] = 0.5
        scenario_risk_params["LinearKnowledgeFunction"]["DimensionName1"]["b"] = 0.72
        scenario_risk_params["LinearKnowledgeFunction"]["DimensionName2"]["a"] = 0.8123
        scenario_risk_params["LinearKnowledgeFunction"]["DimensionName2"]["b"] = 0.32
        scenario_risk_func_def_before = LinearKnowledgeFunctionDefinition(prior_knowledge_region_before, scenario_risk_params)
        scenario_risk_func_before = ScenarioRiskFunction(scenario_risk_func_def_before, 0.3456)
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
        self.assertEqual(scenario_risk_func.normalization_constant, scenario_risk_func_before.normalization_constant)
        self.assertEqual(LinearKnowledgeFunctionDefinition, \
                        type(scenario_risk_func.scenario_risk_function_definition))
        self.assertEqual(prior_knowledge_region_before.definition, \
                        scenario_risk_func.scenario_risk_function_definition.supporting_region.definition)
        print(scenario_risk_func.scenario_risk_function_definition.params.getCondensedParamList())
        list1 = set(scenario_risk_func.scenario_risk_function_definition.params.getCondensedParamList())
        list2 = set(scenario_risk_params.getCondensedParamList() )
        self.assertEqual(list1, list2)
        unpickled_params = scenario_risk_func.scenario_risk_function_definition.params
        self.assertAlmostEqual(unpickled_params.getReal("LinearKnowledgeFunction::DimensionName1::a", "", 1.0), 0.5, 5)
        self.assertAlmostEqual(unpickled_params.getReal("LinearKnowledgeFunction::DimensionName1::b", "", 1.0), 0.72, 5)
        self.assertAlmostEqual(unpickled_params.getReal("LinearKnowledgeFunction::DimensionName2::a", "", 1.0), 0.8123, 5)
        self.assertAlmostEqual(unpickled_params.getReal("LinearKnowledgeFunction::DimensionName2::b", "", 1.0), 0.32, 5)


if __name__ == '__main__':
    unittest.main()