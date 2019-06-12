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
    CStateMachineController smc(1, 1, states);
    results = smc.activate(inputs);

    assert(results.size() == 2);
    assert(results[0] == 0.0);
    assert(results[1] > 0.76 && results[1] < 0.77);
}

void condition_test()
{
    CCondition cond(1, 0, 2.0);
    assert(!cond.evaluate({0.0, 0.0}));

    CCondition cond1(1, 0, 2.0);
    assert(cond1.evaluate({0.0, 2.0}));

    CCondition cond2(1, 0, 2.0);
    assert(!cond2.evaluate({2.0, 0.0}));

    CCondition cond3(0, 1, 0.5);
    assert(!cond3.evaluate({0.0, 0.0}));

    CCondition cond4(0, 1, 0.5);
    assert(cond4.evaluate({2.0, 0.0}));

    CCondition cond5(0, 2, 0.5);
    assert(cond5.evaluate({0.0, 0.0}));

    CCondition cond6(0, 2, 0.5);
    assert(!cond6.evaluate({2.0, 0.0}));

    CCondition cond7(0, 3, 0.5);
    assert(!cond7.evaluate({2.0, 0.0}));

    CCondition cond8(2, 2, 0.5);
    assert(!cond8.evaluate({2.0, 0.0}));

    std::cout << "Passed condition test" << std::endl;
}

void transition_test()
{
    std::vector<CCondition> conditions;
    CStateTransition transition(21, 22, false, conditions);
    assert(transition.m_iEndState == 22);
    assert(transition.m_iBeginState == 21);
    assert(!transition.evaluate({0.0, 0.5}));

    CStateTransition transition1(21, 22, true, conditions);
    assert(transition1.evaluate({0.0, 0.5}));

    // Check whether changing the vector also changes the transition.
    conditions.emplace_back(CCondition(1, 0, 2.0));
    assert(transition1.evaluate({0.0, 0.5}));

    // Check that a false condition does not evaluate the transition.
    CStateTransition transition2(21, 22, true, conditions);
    assert(!transition2.evaluate({0.0, 0.5}));

    // Check that a true condition does evaluate the transition.
    conditions.clear();
    conditions.emplace_back(CCondition(1, 0, 0.5));
    CStateTransition transition3(21, 22, true, conditions);
    assert(transition3.evaluate({0.0, 0.5}));

    // Check that a true and a false condition does not evaluate the transition.
    conditions.clear();
    conditions.emplace_back(CCondition(1, 0, 0.5));
    conditions.emplace_back(CCondition(0, 0, 0.5));
    CStateTransition transition4(21, 22, true, conditions);
    assert(!transition4.evaluate({0.0, 0.5}));

    // Check that a true and a true condition does evaluate the transition.
    conditions.clear();
    conditions.emplace_back(CCondition(1, 0, 0.5));
    conditions.emplace_back(CCondition(0, 1, 0.5));
    CStateTransition transition5(21, 22, true, conditions);
    assert(!transition5.evaluate({0.0, 0.5}));

    // Check that 3x true conditions do evaluate
    conditions.clear();
    conditions.emplace_back(CCondition(1, 1, 0.5));
    conditions.emplace_back(CCondition(0, 2, 0.5));
    conditions.emplace_back(CCondition(0, 0, 0.0));
    CStateTransition transition6(21, 22, true, conditions);
    assert(transition6.evaluate({0.0, 1.0}));

    // Check that 3x false conditions do evaluate
    conditions.clear();
    conditions.emplace_back(CCondition(1, 2, 0.5));
    conditions.emplace_back(CCondition(0, 1, 0.5));
    conditions.emplace_back(CCondition(0, 0, 0.1));
    CStateTransition transition7(21, 22, true, conditions);
    assert(!transition7.evaluate({0.0, 1.0}));

    // Check that true, true, false condition does evaluate
    conditions.clear();
    conditions.emplace_back(CCondition(1, 1, 0.5));
    conditions.emplace_back(CCondition(0, 2, 0.5));
    conditions.emplace_back(CCondition(0, 0, 0.1));
    CStateTransition transition8(21, 22, true, conditions);
    assert(!transition8.evaluate({0.0, 1.0}));

    std::cout << "Passed transition test" << std::endl;
}

void transitioned_state_test()
{
    std::vector<CCondition> conditions;

    std::vector<Real> weight_1{1.0};
    std::vector<Real> weight_2{-0.5};
    std::vector<std::vector<Real>> weights{weight_1, weight_2};
    std::vector<Real> biases{0.0, 1.0};
    std::string actfunc = "tanh";
    std::string aggfunc = "sum";
    CPerceptronNetwork pn(weights, biases, actfunc, aggfunc);

    // Test no transitions.
    CTransitionedState state(42, pn);
    assert(state.next_state({0.0}) == 42);

    // Test True transition, no condition
    state.add_transition(CStateTransition(42, 43, true, conditions));
    assert(state.next_state({0.0}) == 43);

    // Test False transition
    CTransitionedState state1(42, pn);
    conditions.clear();
    conditions.emplace_back(CCondition(0, 0, 0.1));
    state1.add_transition(CStateTransition(42, 43, true, conditions));
    assert(state1.next_state({0.0}) == 42);

    // Test true transition, one condition
    CTransitionedState state2(42, pn);
    conditions.clear();
    conditions.emplace_back(CCondition(0, 0, 0.0));
    state2.add_transition(CStateTransition(42, 43, true, conditions));
    assert(state2.next_state({0.0}) == 43);

    // Test multiple false transitions
    CTransitionedState state3(42, pn);
    conditions.clear();
    conditions.emplace_back(CCondition(0, 0, 0.1));
    state3.add_transition(CStateTransition(42, 43, true, conditions));
    state3.add_transition(CStateTransition(42, 44, true, conditions));
    assert(state3.next_state({0.0}) == 42);

    std::cout << "Transition state machine test passed." << std::endl;
}