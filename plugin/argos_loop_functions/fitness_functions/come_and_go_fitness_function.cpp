//
// Created by matthijs on 23-7-19.
//

#include "come_and_go_fitness_function.h"
#include <argos3/core/simulator/simulator.h>
#include <argos3/core/simulator/entity/positional_entity.h>

#define IN_RANGE_DISTANCE 0.1

CComeAndGoFitnessFunction::CComeAndGoFitnessFunction(CFootBotEntity& robot_entity)
    : m_cRobotEntity(robot_entity), m_bEncounteredLight(false)
{}

void CComeAndGoFitnessFunction::post_step_evaluation()
{
    CPositionalEntity& light = (CPositionalEntity&) CSimulator::GetInstance().GetSpace().GetEntity("light");
    CVector3 robotPosition = m_cRobotEntity.GetEmbodiedEntity().GetOriginAnchor().Position;
    CVector3 lightPosition = light.GetPosition();
    lightPosition.SetZ(robotPosition.GetZ());
    CVector3 differenceVector = robotPosition - lightPosition;

    if  (differenceVector.Length() < IN_RANGE_DISTANCE)
    {   // If the robot is close enough to the light, then swap the fitness score.
        m_bEncounteredLight = true;
    }

    // Let the score be the distance of the robot to the light, if it has been at the light, 0.0 otherwise.
    m_fScore = m_bEncounteredLight ? 1.0 + differenceVector.Length() : 0.0;
}

void CComeAndGoFitnessFunction::reset()
{
    CFitnessFunction::reset();
    m_bEncounteredLight = false;
}