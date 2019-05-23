#include "mpga_phototaxis_obstacle_loop_functions.h"
#include "../argos_ros_bot/argos_ros_bot.h"
#include <argos3/core/simulator/simulator.h>
#include <argos3/core/simulator/entity/positional_entity.h>
#include <argos3/core/utility/math/vector3.h>
#include <cmath>

/****************************************/
/****************************************/

using namespace argos;

#define FITNESS_POWER 3            // This define is used to indicate the shape of the fitness function.

const Real maxDistance = 11.0;      // Constant used to indicate the max distance from the robot to the light.

CMPGAPhototaxisObstacleLoopFunctions::CMPGAPhototaxisObstacleLoopFunctions() : m_fScore(0), m_fMaxDistance(maxDistance)
{ }

void CMPGAPhototaxisObstacleLoopFunctions::SetStartLocation() {

    m_vecResetLocation.Position.SetX(0.0);
    m_vecResetLocation.Position.SetY(2.5);

    /* Set orientation */
    CRadians cOrient = m_pcRNG->Uniform(CRadians::UNSIGNED_RANGE);
    m_vecResetLocation.Orientation.FromEulerAngles(
            cOrient,        // rotation around Z
            CRadians::ZERO, // rotation around Y
            CRadians::ZERO  // rotation around X
    );
}

/****************************************/
/****************************************/

void CMPGAPhototaxisObstacleLoopFunctions::Reset() {

    CMPGAPhototaxisLoopFunctions::Reset();

    m_fScore = 0;
}

/****************************************/
/****************************************/

void CMPGAPhototaxisObstacleLoopFunctions::PostStep() {

    CMPGAPhototaxisLoopFunctions::PostStep();

    if (!m_bDone)
    {
        // After a collision do not update the score anymore.
        if (m_pcFootBot->GetEmbodiedEntity().IsCollidingWithSomething())
        {
            m_bDone = true;
        }

        m_fScore += CalculateStepScore();
    }
}

/****************************************/
/****************************************/

Real CMPGAPhototaxisObstacleLoopFunctions::Score() {
   /* The performance is simply the distance of the robot to the origin */

    return m_fScore;
}

Real CMPGAPhototaxisObstacleLoopFunctions::CalculateStepScore()
{
    CPositionalEntity& light = (CPositionalEntity&) CSimulator::GetInstance().GetSpace().GetEntity("light");
    CVector3 robotPosition = m_pcFootBot->GetEmbodiedEntity().GetOriginAnchor().Position;
    CVector3 lightPosition = light.GetPosition();
    lightPosition.SetZ(robotPosition.GetZ());
    CVector3 differenceVector = robotPosition - lightPosition;

    // Get points for closeness to light, as long as the robot did not collide.
    return calculateFitness(differenceVector.Length());
}

Real CMPGAPhototaxisObstacleLoopFunctions::calculateFitness(Real distance)
{
    Real normalizedDistance = 1.0 - distance / m_fMaxDistance;
    return pow(normalizedDistance, FITNESS_POWER);
}


/****************************************/
/****************************************/

REGISTER_LOOP_FUNCTIONS(CMPGAPhototaxisObstacleLoopFunctions, "mpga_phototaxis_obstacle_loop_functions")
