//
// Created by matthijs on 2-7-19.
//

#include "summed_in_light_range_fitness_function.h"

CSummedInLightRangeFitnessFunction::CSummedInLightRangeFitnessFunction(CFootBotEntity &robot_entity,
        Real in_range_distance)
    : m_cRobotEntity(robot_entity), m_fInRangeDistance(in_range_distance)
{}

void CSummedInLightRangeFitnessFunction::post_step_evaluation()
{
    CPositionalEntity& light = (CPositionalEntity&) CSimulator::GetInstance().GetSpace().GetEntity("light");
    CVector3 robotPosition = m_cRobotEntity.GetEmbodiedEntity().GetOriginAnchor().Position;
    CVector3 lightPosition = light.GetPosition();
    lightPosition.SetZ(robotPosition.GetZ());
    CVector3 differenceVector = robotPosition - lightPosition;

    if (differenceVector.Length() < m_fInRangeDistance)
    {
        m_fScore += 1.0;
    }
}
