//
// Created by matthijs on 23-7-19.
//

#include "come_and_go_loop_function.h"
#include "fitness_functions/come_and_go_fitness_function.h"

void CComeAndGoLoopFunction::Init(TConfigurationNode &t_node)
{
    CGenomeRunnerLoopFunction::Init(t_node);

    delete m_pFitnessFunction;
    m_pFitnessFunction = new CComeAndGoFitnessFunction(*m_pcFootBot);
}

/****************************************/
/****************************************/

REGISTER_LOOP_FUNCTIONS(CComeAndGoLoopFunction, "come_and_go_loop_functions")
