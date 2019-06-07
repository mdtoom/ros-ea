//
// Created by matthijs on 7-6-19.
//

#ifndef MA_EVOLUTION_TRANSITION_STATE_MACHINE_H
#define MA_EVOLUTION_TRANSITION_STATE_MACHINE_H

#include "perceptron_network.h"
#include "state_machine_controller.h"

/** This class gives what a transition looks like in a state transition ruled state machine. */
class CStateTransition
{

};

/**
 * This class is gives a state in a state machine controller that switches state using transitions.
 */
class CTransitionedState : public CControllerState {

public:
    CTransitionedState(int state_id, const CPerceptronNetwork pn);

    void add_transition(CStateTransition transition);

    virtual int next_state(const std::vector<Real> inputs) override;

private:
    std::vector<CStateTransition> m_vTransitions;
};


#endif //MA_EVOLUTION_TRANSITION_STATE_MACHINE_H
