//
// Created by matthijs on 1-7-19.
//

#include "light_random_init_runner.h"

#include <argos3/core/utility/math/range.h>
#include <argos3/plugins/simulator/entities/light_entity.h>
#include "../argos_ros_bot/argos_ros_bot.h"
#include "ma_evolution/SimulationReport.h"

CLightRandomInitLoopFunction::CLightRandomInitLoopFunction()
    : m_iMaxNumTrials(5), m_iNumTrials(0), m_fCurrentControllerScore(0.0)
{ }

void CLightRandomInitLoopFunction::Init(TConfigurationNode &t_node)
{
    CGenomeRunnerLoopFunction::Init(t_node);

    // Store the light position.
    CLightEntity& cLight = dynamic_cast<CLightEntity&>(GetSpace().GetEntity("light"));
    m_vLightPosition = cLight.GetPosition();

    GetNodeAttributeOrDefault(t_node, "num_trials", m_iMaxNumTrials, m_iMaxNumTrials);
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

void CLightRandomInitLoopFunction::finish_simulation_iteration()
{
    gather_controller_states();
    CArgosRosBot &controller = (CArgosRosBot&) m_pcFootBot->GetControllableEntity().GetController();
    CRobotController *current_controller = controller.get_controller();
    // Add the score of this run to the current score.
    m_fCurrentControllerScore += m_pFitnessFunction->get_fitness();
    // Reset the simulation for the next iteration.
    Reset();
    m_iNumTrials++;

    if (m_iNumTrials >= m_iMaxNumTrials)
    {
        // Calculate the average score.
        Real score = m_fCurrentControllerScore / m_iNumTrials;

        // publish the score in the score topic.
        ma_evolution::SimulationReport scoreMsg;
        scoreMsg.key = current_controller->m_iID;
        scoreMsg.gen_hash = current_controller->m_iGenerationID;
        scoreMsg.score = score;
        m_pcScorePublisher.publish(scoreMsg);

        controller.set_controller(nullptr);     // Set that no controller is currently on the robot.
        delete current_controller;     // Delete the old controller

        m_iNumTrials = 0;
        m_fCurrentControllerScore = 0.0;
    }
}

/****************************************/
/****************************************/

REGISTER_LOOP_FUNCTIONS(CLightRandomInitLoopFunction, "random_light_runner_loop_functions")
