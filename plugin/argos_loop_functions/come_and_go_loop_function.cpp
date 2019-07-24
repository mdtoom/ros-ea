//
// Created by matthijs on 23-7-19.
//

#include "come_and_go_loop_function.h"
#include "fitness_functions/come_and_go_fitness_function.h"

#include <argos3/core/utility/math/vector3.h>
#include <argos3/core/utility/math/angles.h>

void CComeAndGoLoopFunction::Init(TConfigurationNode &t_node)
{
    CGenomeRunnerLoopFunction::Init(t_node);

    delete m_pFitnessFunction;
    m_pFitnessFunction = new CComeAndGoFitnessFunction(*m_pcFootBot);
}

void CComeAndGoLoopFunction::Reset()
{
    CGenomeRunnerLoopFunction::Reset();

    // Reset the robot at a random location.
    Real y_offset = m_pcRNG->Uniform(CRange<Real>(-3.0, 3.0));
    Real x_offset = m_pcRNG->Uniform(CRange<Real>(-1.5, 1.5));

    CVector3 new_position(m_vecResetLocation.Position[0] + x_offset, m_vecResetLocation.Position[1] + y_offset,
            m_vecResetLocation.Position[0]);

    // Randomize the heading.
    CRadians random_heading = m_pcRNG->Uniform(CRange<CRadians>(-CRadians::PI_OVER_TWO, CRadians::PI_OVER_TWO));
    m_vecResetLocation.Orientation.FromEulerAngles(random_heading, CRadians::ZERO, CRadians::ZERO);

    if(!MoveEntity(m_pcFootBot->GetEmbodiedEntity(), new_position, m_vecResetLocation.Orientation, false))
    {
        LOGERR << "Can't move robot in <" << new_position
               << ">, <" << m_vecResetLocation.Orientation << ">" << std::endl;
    }

}

/****************************************/
/****************************************/

REGISTER_LOOP_FUNCTIONS(CComeAndGoLoopFunction, "come_and_go_loop_functions")
