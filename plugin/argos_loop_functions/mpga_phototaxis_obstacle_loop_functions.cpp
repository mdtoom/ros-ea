#include "mpga_phototaxis_obstacle_loop_functions.h"
#include "../argos_ros_bot/argos_ros_bot.h"
#include <argos3/core/simulator/simulator.h>
#include <argos3/core/simulator/entity/positional_entity.h>
#include <argos3/core/utility/math/vector3.h>

/****************************************/
/****************************************/

using namespace argos;

CMPGAPhototaxisObstacleLoopFunctions::CMPGAPhototaxisObstacleLoopFunctions() : m_iScore(0)
{ }

void CMPGAPhototaxisObstacleLoopFunctions::SetStartLocation() {

    m_vecResetLocation.Position.SetX(0);
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

    m_iScore = 0;
}

/****************************************/
/****************************************/

void CMPGAPhototaxisObstacleLoopFunctions::PostStep() {

    CMPGAPhototaxisLoopFunctions::PostStep();

    CPositionalEntity& light = (CPositionalEntity&) CSimulator::GetInstance().GetSpace().GetEntity("light");
    CVector3 robotPosition = m_pcFootBot->GetEmbodiedEntity().GetOriginAnchor().Position;
    CVector3 lightPosition = light.GetPosition();
    lightPosition.SetZ(robotPosition.GetZ());
    CVector3 differenceVector = robotPosition - lightPosition;

    if (!m_bDone)
    {
        // After a collision do not update the score anymore.
        if (m_pcFootBot->GetEmbodiedEntity().IsCollidingWithSomething())
        {
            m_bDone = true;
        }

        // Get points for closeness to light, as long as the robot did not collide.
        Real normalizedDistance = (10.0 - differenceVector.Length()) / 10.0;
        if (normalizedDistance > 1.0 or normalizedDistance < 0.0) {
            LOG << "Got distance: " << normalizedDistance << std::endl;
        }
        m_iScore += normalizedDistance * normalizedDistance;
    }
}

/****************************************/
/****************************************/

Real CMPGAPhototaxisObstacleLoopFunctions::Score() {
   /* The performance is simply the distance of the robot to the origin */

    return m_iScore;
}

/****************************************/
/****************************************/

REGISTER_LOOP_FUNCTIONS(CMPGAPhototaxisObstacleLoopFunctions, "mpga_phototaxis_obstacle_loop_functions")
