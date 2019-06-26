//
// Created by matthijs on 26-6-19.
//

#include "fitness_evaluating_loop_function.h"

#include "fitness_functions/space_dependent_fitness_function.h"

#define FITNESS_POWER_DEFAULT 3

using namespace argos;

CFitnessEvaluatingLoopFunction::CFitnessEvaluatingLoopFunction() : m_pFitnessFunction(nullptr)
{}

CFitnessEvaluatingLoopFunction::~CFitnessEvaluatingLoopFunction()
{
    delete m_pFitnessFunction;
}

void CFitnessEvaluatingLoopFunction::Init(TConfigurationNode& t_node)
{
    CRobotLaunchingLoopFunction::Init(t_node);

    // Set the power attribute which is used for deciding the power of the fitness calculation.
    Real fitness_power = FITNESS_POWER_DEFAULT;
    GetNodeAttributeOrDefault(t_node, "fitness_power", fitness_power, fitness_power);

    m_pFitnessFunction = new CSpaceDependentFitnessFunction(*m_pcFootBot, fitness_power, *this);

    // Register the get score service of the node of the simulation.
    m_pcScoreService = nodeHandle->
            advertiseService("score", &CFitnessEvaluatingLoopFunction::GetScore, this);

    LOG << "Fitness evaluation loop function initialized" << std::endl;
    LOG.Flush();
}


/****************************************/
/****************************************/

void CFitnessEvaluatingLoopFunction::Reset()
{
    CRobotLaunchingLoopFunction::Reset();
    m_pFitnessFunction->reset();
}

/****************************************/
/****************************************/

bool CFitnessEvaluatingLoopFunction::GetScore(ma_evolution::SimScore::Request& request,
                                                    ma_evolution::SimScore::Response& response)
{
    Real score = m_pFitnessFunction->get_fitness();
    response.score = (float) score;
    return true;
}

void CFitnessEvaluatingLoopFunction::PostStep()
{
    CRobotLaunchingLoopFunction::PostStep();
    m_pFitnessFunction->post_step_evaluation();
}
