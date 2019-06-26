//
// Created by matthijs on 26-6-19.
//

#include "absolute_distance_fitness_function.h"
#include <argos3/core/simulator/simulator.h>
#include <argos3/core/simulator/entity/positional_entity.h>

using namespace argos;

const Real maxDistance = 11.0;      // Constant used to indicate the max distance from the robot to the light.

CAbsoluteDistanceFitnessFunction::CAbsoluteDistanceFitnessFunction(CFootBotEntity& robot_entity, Real fitness_power)
        : CFitnessFunction(), m_cRobotEntity(robot_entity), m_fFitnessPower(fitness_power), m_fMaxDistance(maxDistance)
{}

void CAbsoluteDistanceFitnessFunction::post_step_evaluation()
{
    CPositionalEntity& light = (CPositionalEntity&) CSimulator::GetInstance().GetSpace().GetEntity("light");
    CVector3 robotPosition = m_cRobotEntity.GetEmbodiedEntity().GetOriginAnchor().Position;
    CVector3 lightPosition = light.GetPosition();
    lightPosition.SetZ(robotPosition.GetZ());
    CVector3 differenceVector = robotPosition - lightPosition;

    // Get points for closeness to light, as long as the robot did not collide.
    m_fScore += calculate_distance_based_fitness(differenceVector.Length());
}

Real CAbsoluteDistanceFitnessFunction::calculate_distance_based_fitness(Real distance)
{
    Real normalizedDistance = 1.0 - distance / m_fMaxDistance;
    return pow(normalizedDistance, m_fFitnessPower);
}
