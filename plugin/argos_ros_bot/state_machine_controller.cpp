//
// Created by matthijs on 7-6-19.
//

#include "state_machine_controller.h"


CControllerState::CControllerState(int state_id, const CPerceptronNetwork pn) :
        m_iStateId(state_id), m_cPerceptronNetwork(pn)
{ }

std::vector<Real> CControllerState::activate(const std::vector<Real> inputs)
{
    return m_cPerceptronNetwork.activate(inputs);
}

CStateMachineController::CStateMachineController(int controller_id, int gen_id, std::vector<CControllerState*> states) :
    CRobotController(controller_id, gen_id), m_iCurrentState(0), m_vStates(states)
{ }

CStateMachineController::~CStateMachineController()
{
    for (std::vector<CControllerState*>::iterator it = m_vStates.begin(); it != m_vStates.end(); ++it)
    {
        delete *it;
    }

    m_vStates.clear();
}

std::vector<Real> CStateMachineController::activate(const std::vector <Real> inputs)
{
    m_iCurrentState = current_state()->next_state(inputs);
    m_vStateHistory.emplace_back(m_iCurrentState);
    return current_state()->activate(inputs);
}

CControllerState *CStateMachineController::current_state()
{
    return m_vStates[m_iCurrentState];
}