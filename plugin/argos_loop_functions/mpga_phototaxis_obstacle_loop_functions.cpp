#include "mpga_phototaxis_obstacle_loop_functions.h"

#include <argos3/core/simulator/simulator.h>
#include <argos3/core/simulator/entity/positional_entity.h>
#include <argos3/core/utility/math/vector3.h>
#include <cmath>
#include <iostream>
#include <sstream>
#include <ros/callback_queue.h>
#include "message_decoder.h"
#include "../argos_ros_bot/argos_ros_bot.h"
#include "ma_evolution/Score.h"

/****************************************/
/****************************************/

using namespace argos;

#define FITNESS_POWER_DEFAULT 3            // This define is used to indicate the shape of the fitness function.

const Real maxDistance = 11.0;      // Constant used to indicate the max distance from the robot to the light.

CMPGAPhototaxisObstacleLoopFunctions::CMPGAPhototaxisObstacleLoopFunctions() : CRobotLaunchingLoopFunction(),
    m_fScore(0), m_fMaxDistance(maxDistance), m_fFitnessPower(FITNESS_POWER_DEFAULT),
    m_cGenomeBuffer(NULL), m_iExecutedSteps(0), m_iTargetExecutedSteps(600)
{ }

/****************************************/
/****************************************/

CMPGAPhototaxisObstacleLoopFunctions::~CMPGAPhototaxisObstacleLoopFunctions()
{
    delete m_cGenomeBuffer;
}

/****************************************/
/****************************************/

void CMPGAPhototaxisObstacleLoopFunctions::Init(TConfigurationNode& t_node)
{
    CRobotLaunchingLoopFunction::Init(t_node);

    // Set the power attribute which is used for deciding the power of the fitness calculation.
    GetNodeAttributeOrDefault(t_node, "fitness_power", m_fFitnessPower, m_fFitnessPower);

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

    // Register the get score service of the node of the simulation.
    m_pcScoreService = nodeHandle->
            advertiseService("score", &CMPGAPhototaxisObstacleLoopFunctions::GetScore, this);

    m_pcScorePublisher = nodeHandle->advertise<ma_evolution::Score>("score_topic", 100);

    LOG << "genome receiving loop function initialized" << std::endl;
    LOG.Flush();
}

/****************************************/
/****************************************/

void CMPGAPhototaxisObstacleLoopFunctions::Reset()
{
    CRobotLaunchingLoopFunction::Reset();
    m_fScore = 0;
    m_iExecutedSteps = 0;
}

/****************************************/
/****************************************/

bool CMPGAPhototaxisObstacleLoopFunctions::GetScore(ma_evolution::SimScore::Request& request,
                                            ma_evolution::SimScore::Response& response)
{
    Real score = Score();
    response.score = (float) score;
    return true;
}

/****************************************/
/****************************************/

void CMPGAPhototaxisObstacleLoopFunctions::PreStep()
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

void CMPGAPhototaxisObstacleLoopFunctions::PostStep()
{
    CRobotLaunchingLoopFunction::PostStep();

    m_fScore += CalculateStepScore();
    m_iExecutedSteps++;

    if (m_pcFootBot->GetEmbodiedEntity().IsCollidingWithSomething() || m_iExecutedSteps >= m_iTargetExecutedSteps)
    {   // If the execution is finished.
        gather_controller_states();
        CArgosRosBot &controller = (CArgosRosBot&) m_pcFootBot->GetControllableEntity().GetController();
        CRobotController *current_controller = controller.get_controller();

        // publish the score in the score topic.
        ma_evolution::Score scoreMsg;
        scoreMsg.key = current_controller->m_iID;
        scoreMsg.gen_hash = current_controller->m_iGenerationID;
        scoreMsg.score = m_fScore;
        m_pcScorePublisher.publish(scoreMsg);

        controller.set_controller(nullptr);     // Set that no controller is currently on the robot.
        delete current_controller;     // Delete the old controller
    }
}

/****************************************/
/****************************************/

Real CMPGAPhototaxisObstacleLoopFunctions::Score() {
   /* The performance is simply the distance of the robot to the origin */

    return m_fScore;
}

/****************************************/
/****************************************/

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
    return pow(normalizedDistance, m_fFitnessPower);
}


/****************************************/
/****************************************/

REGISTER_LOOP_FUNCTIONS(CMPGAPhototaxisObstacleLoopFunctions, "mpga_phototaxis_obstacle_loop_functions")
