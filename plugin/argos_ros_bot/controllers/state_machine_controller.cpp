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

CStateMachineController::CStateMachineController(const ControllerHeader header,
        std::vector<CControllerState*> states)
    : CRobotController(header), m_iCurrentState(states[0]->m_iStateId), m_vStates(states)
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
    return get_state(m_iCurrentState);
}

CControllerState *CStateMachineController::get_state(int state_id)
{
    // Find the state with the current id.
    for (std::vector<CControllerState*>::iterator it = m_vStates.begin(); it != m_vStates.end(); ++it)
    {
        if ((*it)->m_iStateId == state_id)
        {
            return *it;
        }
    }

    std::cout << "Current state id " << m_iCurrentState << " not found." << std::endl;
    return nullptr;
}