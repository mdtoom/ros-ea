//
// Created by matthijs on 7-6-19.
//

#ifndef MA_EVOLUTION_TRANSITION_STATE_MACHINE_H
#define MA_EVOLUTION_TRANSITION_STATE_MACHINE_H

#include "perceptron_network.h"
#include "state_machine_controller.h"

class CCondition
{
public:

    CCondition(int sensor_index, int op, Real comparator);

    /**
     * This function evaluates whether the condition is true.
     * @param sensor_inputs     - Sensor inputs of the transition.
     * @return                  - True, if the current transition is valid.
     */
    bool evaluate(std::vector<Real> sensor_inputs);

private:

    int m_iSensorIndex;
    int m_iOperator;
    Real m_fComparator;

};

/** This class gives what a transition looks like in a state transition ruled state machine. */
class CStateTransition
{

public:

    CStateTransition(int begin_state, int end_state, bool enabled, std::vector<CCondition> conditions);

    /**
     * This function evaluates whether the transition should be taken.
     * @param sensor_inputs     - Sensor inputs of the transition.
     * @return                  - True, if the current transition is valid.
     */
    bool evaluate(std::vector<Real> sensor_inputs);

    int m_iBeginState;
    int m_iEndState;

private:
    std::vector<CCondition> m_vConditions;

    bool m_bEnabled;

};


/**
 * This class is gives a state in a state machine controller that switches state using transitions.
 */
class CTransitionedState : public CControllerState {

public:
    CTransitionedState(int state_id, const CPerceptronNetwork pn);

    void add_transition(const CStateTransition &transition);

    virtual int next_state(const std::vector<Real> inputs) override;

private:
    std::vector<CStateTransition> m_vTransitions;
};


#endif //MA_EVOLUTION_TRANSITION_STATE_MACHINE_H
