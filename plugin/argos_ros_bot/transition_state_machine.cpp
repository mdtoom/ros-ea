//
// Created by matthijs on 7-6-19.
//

#include "transition_state_machine.h"
#include <cmath>
#include <cassert>

CCondition::CCondition(int sensor_index, int op, Real comparator)
            : m_iSensorIndex(sensor_index), m_iOperator(op), m_fComparator(comparator)
{ }

// This function is used for float compare with 4 decimals.
bool cmpf(float A, float B, float epsilon = 0.005f)
{
    return (fabs(A - B) < epsilon);
}

bool CCondition::evaluate(std::vector<Real> sensor_inputs)
{
    if (m_iSensorIndex >= sensor_inputs.size())
    {
        std::cout << "Condition uses sensor that isn't available" << std::endl;
        return false;
    }

    Real sensor_value = sensor_inputs[m_iSensorIndex];

    switch (m_iOperator) {
        case 0:
            return cmpf(sensor_value, m_fComparator);
        case 1:
            return sensor_value > m_fComparator;
        case 2:
            return sensor_value < m_fComparator;
        default:
            std::cout << "Invalid sensor comparator " << m_iOperator << std::endl;
            return false;
    }
}

CStateTransition::CStateTransition(int begin_state, int end_state, bool enabled,  bool or_comparison,
                                   std::vector <CCondition> conditions)
    : m_iBeginState(begin_state), m_iEndState(end_state), m_vConditions(conditions), m_bEnabled(enabled),
        m_bOrComparison(or_comparison)
{ }

bool CStateTransition::evaluate(std::vector <Real> sensor_inputs)
{
    bool result;
    if (m_bOrComparison)
    {
        result = evaluate_or(sensor_inputs);
    } else {
        result = evaluate_and(sensor_inputs);
    }

    return result && m_bEnabled;        // Only true when the transition is enabled.
}

bool CStateTransition::evaluate_and(std::vector <Real> sensor_inputs)
{
    // Transition is taken if enabled and all conditions are true (or there are no conditions)
    bool transition_taken = true;
    for (std::vector<CCondition>::iterator it = m_vConditions.begin(); it != m_vConditions.end(); ++it)
    {
        transition_taken &= it->evaluate(sensor_inputs);
    }
    return transition_taken;
}


bool CStateTransition::evaluate_or(std::vector <Real> sensor_inputs)
{
    bool transition_taken = false;
    for (auto condition : m_vConditions)
    {
        transition_taken |= condition.evaluate(sensor_inputs);
    }
    return transition_taken;
}


CTransitionedState::CTransitionedState(int state_id, const CPerceptronNetwork pn) :
        CControllerState(state_id, pn)
{ }

void CTransitionedState::add_transition(const CStateTransition &transition)
{
    assert(transition.m_iBeginState ==  m_iStateId);
    m_vTransitions.emplace_back(transition);
}

int CTransitionedState::next_state(const std::vector<Real> inputs)
{
    std::vector<int> available_states;
    for (std::vector<CStateTransition>::iterator it = m_vTransitions.begin(); it != m_vTransitions.end(); ++it)
    {
        if (it->evaluate(inputs))
        {
            available_states.emplace_back(it->m_iEndState);
        }
    }

    int next_state = m_iStateId;
    if (available_states.size() > 0)
    {
        next_state = available_states[rand() % available_states.size()];
    }
    return next_state;
}