//
// Created by matthijs on 7-6-19.
//

#include "transition_state_machine.h"

CTransitionedState::CTransitionedState(int state_id, const CPerceptronNetwork pn) :
        CControllerState(state_id, pn)
{ }


void CTransitionedState::add_transition(CStateTransition transition)
{
    m_vTransitions.emplace_back(transition);
}

int CTransitionedState::next_state(const std::vector<Real> inputs)
{
    for (std::vector<CStateTransition>::iterator it = m_vTransitions.begin(); it != m_vTransitions.end(); ++it)
    {
        // TODO : implement condition extraction.
    }

    return m_iStateId;
}