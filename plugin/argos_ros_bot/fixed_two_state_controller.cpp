//
// Created by matthijs on 24-6-19.
//

#include "fixed_two_state_controller.h"

#define IN_RANGE_DISTANCE 0.5

CFixedTwoStateController::CFixedTwoStateController(int controller_id, int gen_id, const CTransitionedState &state1,
        const CTransitionedState &state2) : CRobotController(controller_id, gen_id), m_cState1(state1), m_cState2(state2)
{ }

std::vector<Real> CFixedTwoStateController::activate(const std::vector<Real> inputs)
{
    std::vector<Real> outputs;
    if (object_in_range(inputs))
    {
        outputs = m_cState2.activate(inputs);
    } else {
        outputs = m_cState1.activate(inputs);
    }
    return outputs;
}

bool CFixedTwoStateController::object_in_range(const std::vector<Real> inputs)
{
    // The first 12 sensor readings are the proximity, so these should be within range.
    Real min_value = 1.0;
    for (int i = 0; i < 12; i++)
    {
        if (inputs[i] < min_value && inputs[i] != 0.0)
        {
            min_value = inputs[i];
        }
    }
    return min_value < IN_RANGE_DISTANCE;
}