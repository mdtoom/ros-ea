//
// Created by matthijs on 1-7-19.
//

#include "light_random_init_runner.h"

#include <argos3/core/utility/math/range.h>
#include <argos3/plugins/simulator/entities/light_entity.h>


void CLightRandomInitLoopFunction::Init(TConfigurationNode &t_node)
{
    CGenomeRunnerLoopFunction::Init(t_node);

    // Store the light position.
    CLightEntity& cLight = dynamic_cast<CLightEntity&>(GetSpace().GetEntity("light"));
    m_vLightPosition = cLight.GetPosition();

    std::cout << m_vLightPosition << std::endl;
}

void CLightRandomInitLoopFunction::Reset()
{
    CGenomeRunnerLoopFunction::Reset();

    Real x_offset = m_pcRNG->Uniform(CRange<Real>(-1.0, 1.0));
    Real y_offset = m_pcRNG->Uniform(CRange<Real>(-2.0, 2.0));

    // Get a reference to the light
    CLightEntity& cLight = dynamic_cast<CLightEntity&>(GetSpace().GetEntity("light"));
    // Move the light entity to the wanted position
    cLight.SetPosition(CVector3(m_vLightPosition[0] + x_offset, m_vLightPosition[1] + y_offset, m_vLightPosition[2]));
}

/****************************************/
/****************************************/

REGISTER_LOOP_FUNCTIONS(CLightRandomInitLoopFunction, "random_light_runner_loop_functions")
