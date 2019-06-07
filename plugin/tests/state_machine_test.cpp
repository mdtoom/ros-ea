//
// Created by matthijs on 7-6-19.
//

#include <cassert>
#include <vector>
#include "state_machine_test.h"
#include "../argos_ros_bot/transition_state_machine.h"


void state_machine_test_1_state()
{
    std::vector<Real> weight_1{1.0};
    std::vector<Real> weight_2{-0.5};
    std::vector<std::vector<Real>> weights{weight_1, weight_2};
    std::vector<Real> biases{0.0, 1.0};
    std::string actfunc = "tanh";
    std::string aggfunc = "sum";

    CPerceptronNetwork pn(weights, biases, actfunc, aggfunc);
    CTransitionedState* tsp = new CTransitionedState(0, pn);

    std::vector<Real> inputs{0.0};
    std::vector<Real> results = tsp->activate(inputs);

    assert(results.size() == 2);
    assert(results[0] == 0.0);
    assert(results[1] > 0.76 && results[1] < 0.77);

    std::vector<CControllerState*> states{tsp};
    CStateMachineController smc(states);
    results = smc.activate(inputs);

    assert(results.size() == 2);
    assert(results[0] == 0.0);
    assert(results[1] > 0.76 && results[1] < 0.77);
}