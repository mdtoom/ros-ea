#include "genome_runner_loop_function.h"

#include <iostream>
#include <sstream>
#include <ros/callback_queue.h>
#include "message_decoder.h"
#include "../argos_ros_bot/argos_ros_bot.h"
#include "ma_evolution/Score.h"

/****************************************/
/****************************************/

using namespace argos;

CGenomeRunnerLoopFunction::CGenomeRunnerLoopFunction() : CFitnessEvaluatingLoopFunction(),
    m_cGenomeBuffer(NULL), m_iExecutedSteps(0), m_iTargetExecutedSteps(600)
{ }

/****************************************/
/****************************************/

CGenomeRunnerLoopFunction::~CGenomeRunnerLoopFunction()
{
    delete m_cGenomeBuffer;
}

/****************************************/
/****************************************/

void CGenomeRunnerLoopFunction::Init(TConfigurationNode& t_node)
{
    CFitnessEvaluatingLoopFunction::Init(t_node);

    m_pcScorePublisher = nodeHandle->advertise<ma_evolution::Score>("score_topic", 100);

    // Get the user defined parameters.
    std::string controller_nm;
    nodeHandle->getParam("/controller_type", controller_nm);
    nodeHandle->getParam("/time_steps", m_iTargetExecutedSteps);

    if (!controller_nm.compare("state-machine")) {
        m_cGenomeBuffer = new CGenomeReceiver<ma_evolution::SMGenome>(nodeHandle);
        LOG << "State machine controller selected." << std::endl;
    } else if (!controller_nm.compare("state-selector")) {
        m_cGenomeBuffer = new CGenomeReceiver<ma_evolution::SMSGenome>(nodeHandle);
        LOG << "State selector controller selected." << std::endl;
    } else if (!controller_nm.compare("feed-forward")) {
        m_cGenomeBuffer = new CGenomeReceiver<ma_evolution::NEATGenome>(nodeHandle);
        LOG << "Feed forward controller selected." << std::endl;
    } else if (!controller_nm.compare("fixed-2-states")) {
        m_cGenomeBuffer = new CGenomeReceiver<ma_evolution::SMGenome>(nodeHandle, &decode_fixed_2_states);
        LOG << "Fixed-2-state controller selected." << std::endl;
    } else {
        LOG << "No valid controller selected, exiting." << std::endl;
        LOG.Flush();
        exit(0);
    }

    LOG << "Number of timesteps: " << m_iTargetExecutedSteps << std::endl;
    LOG << "genome receiving loop function initialized" << std::endl;
    LOG.Flush();
}

/****************************************/
/****************************************/

void CGenomeRunnerLoopFunction::Reset()
{
    CFitnessEvaluatingLoopFunction::Reset();
    m_iExecutedSteps = 0;
}

/****************************************/
/****************************************/

void CGenomeRunnerLoopFunction::PreStep()
{
    CArgosRosBot &controller = (CArgosRosBot&) m_pcFootBot->GetControllableEntity().GetController();
    ros::getGlobalCallbackQueue()->callAvailable();

    if (controller.get_controller() == nullptr)
    {   // If no controller is currently evaluated.
        while(!m_cGenomeBuffer->has_next() && ros::ok())
        {   // Wait until a new genome arrives
            ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(1.0));
        }

        Reset();
        CRobotController *new_controller = m_cGenomeBuffer->next();
        controller.set_controller(new_controller);
    }

    if (!ros::ok())
    {
        LOG << "ROS stopped working" << std::endl;
        LOG.Flush();
        exit(0);
    }
}

/****************************************/
/****************************************/

void CGenomeRunnerLoopFunction::PostStep()
{
    CFitnessEvaluatingLoopFunction::PostStep();
    m_iExecutedSteps++;

    if (m_pcFootBot->GetEmbodiedEntity().IsCollidingWithSomething() || m_iExecutedSteps >= m_iTargetExecutedSteps)
    {   // If the execution is finished.
        finish_simulation_iteration();
    }
}

void CGenomeRunnerLoopFunction::finish_simulation_iteration()
{
    gather_controller_states();
    CArgosRosBot &controller = (CArgosRosBot&) m_pcFootBot->GetControllableEntity().GetController();
    CRobotController *current_controller = controller.get_controller();

    // publish the score in the score topic.
    ma_evolution::Score scoreMsg;
    scoreMsg.key = current_controller->m_iID;
    scoreMsg.gen_hash = current_controller->m_iGenerationID;
    scoreMsg.score = m_pFitnessFunction->get_fitness();
    m_pcScorePublisher.publish(scoreMsg);

    controller.set_controller(nullptr);     // Set that no controller is currently on the robot.
    delete current_controller;     // Delete the old controller
}

/****************************************/
/****************************************/

REGISTER_LOOP_FUNCTIONS(CGenomeRunnerLoopFunction, "genome_runner_loop_functions")
