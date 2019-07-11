//
// Created by matthijs on 26-6-19.
//

#include "robot_launching_loop_function.h"
#include "../argos_ros_bot/argos_ros_bot.h"
#include <argos3/core/simulator/entity/positional_entity.h>


// Initialize ROS node.  There will be only one ROS node no matter how many robots are created in
// ARGoS.  However, we will have one instance of the CArgosRosBot class for each ARGoS robot.
ros::NodeHandle* initROS()
{
    int argc = 0;
    char *argv = (char *) "";
    ros::init(argc, &argv, "argos_bridge");
    return new ros::NodeHandle();
}

ros::NodeHandle* CRobotLaunchingLoopFunction::nodeHandle = initROS();

/****************************************/
/****************************************/


CRobotLaunchingLoopFunction::CRobotLaunchingLoopFunction() : m_pcFootBot(NULL)
{ }

CRobotLaunchingLoopFunction::~CRobotLaunchingLoopFunction()
{
    delete m_pcFootBot;
}

void CRobotLaunchingLoopFunction::Init(TConfigurationNode& t_node)
{
    /* Create the random number generator. */
    m_pcRNG = CRandom::CreateRNG("argos");

    SetStartLocation();

    m_pcTrajectoryService = nodeHandle->
            advertiseService("trajectory", &CRobotLaunchingLoopFunction::GetTrajectory, this);

    m_pcStateHistoryService = nodeHandle->
            advertiseService("states_request", &CRobotLaunchingLoopFunction::GetStateHistory, this);

    m_pcAtLightService = nodeHandle->
            advertiseService("at_light", &CRobotLaunchingLoopFunction::GetAtLight, this);

    // Create the foot-bot and get a reference to its controller
    std::string controller_nm = "argos_ros_bot";
    GetNodeAttributeOrDefault(t_node, "argos_controller", controller_nm, controller_nm);
    m_pcFootBot = new CFootBotEntity("fb", controller_nm);
    AddEntity(*m_pcFootBot);
    CRobotLaunchingLoopFunction::Reset();

    LOG << "Robot loop function initialized" << std::endl;
    LOG.Flush();
}

/****************************************/
/****************************************/

void CRobotLaunchingLoopFunction::SetStartLocation() {

    m_vecResetLocation.Position.SetX(0.0);
    m_vecResetLocation.Position.SetY(2.5);

    /* Set orientation */
    CRadians cOrient = m_pcRNG->Uniform(CRadians::UNSIGNED_RANGE);
    m_vecResetLocation.Orientation.FromEulerAngles(
            CRadians::ZERO,        // rotation around Z
            CRadians::ZERO, // rotation around Y
            CRadians::ZERO  // rotation around X
    );
}

/****************************************/
/****************************************/

bool CRobotLaunchingLoopFunction::GetTrajectory(ma_evolution::Trajectory::Request& request,
                                                         ma_evolution::Trajectory::Response& response)
{
    for(std::vector<geometry_msgs::Point>::iterator it = m_vLocations.begin(); it != m_vLocations.end(); ++it)
    {
        response.Locations.emplace_back(*it);
    }

    return true;
}

/****************************************/
/****************************************/

bool CRobotLaunchingLoopFunction::GetStateHistory(ma_evolution::StateRequest::Request &request,
                                                           ma_evolution::StateRequest::Response &response) {

    for(std::vector<int>::iterator it = m_vControllerStates.begin(); it != m_vControllerStates.end(); ++it)
    {
        response.StateSequence.emplace_back(*it);
    }

    return true;
}

/****************************************/
/****************************************/

bool CRobotLaunchingLoopFunction::GetAtLight(ma_evolution::AtLight::Request &request,
                                             ma_evolution::AtLight::Response &response)
{
    response.atLight = at_light();
    return true;
}

/****************************************/
/****************************************/

void CRobotLaunchingLoopFunction::Reset() {

    /*
    * Move robot to the initial position corresponding to the current trial
    */
    if(!MoveEntity(
            m_pcFootBot->GetEmbodiedEntity(),             // move the body of the robot
            m_vecResetLocation.Position,    // to this position
            m_vecResetLocation.Orientation, // with this orientation
            false                                         // this is not a check, leave the robot there
    )) {
        LOGERR << "Can't move robot in <" << m_vecResetLocation.Position
               << ">, <" << m_vecResetLocation.Orientation << ">" << std::endl;
    }

    m_vLocations.clear();
    m_vControllerStates.clear();
}

/****************************************/
/****************************************/

void CRobotLaunchingLoopFunction::PostStep()
{
    // Add the current robot location to the list of visited locations.
    CVector3 robotPosition = m_pcFootBot->GetEmbodiedEntity().GetOriginAnchor().Position;
    geometry_msgs::Point point;
    point.x = robotPosition.GetX();
    point.y = robotPosition.GetY();
    point.z = robotPosition.GetZ();
    m_vLocations.emplace_back(point);
}

void CRobotLaunchingLoopFunction::gather_controller_states()
{
    CArgosRosBot &controller = (CArgosRosBot&) m_pcFootBot->GetControllableEntity().GetController();
    CRobotController *current_controller = controller.get_controller();

    // Copy the states the controller has been in.
    m_vControllerStates.clear();
    for (std::vector<int>::iterator it = current_controller->m_vStateHistory.begin();
         it != current_controller->m_vStateHistory.end(); ++it)
    {
        m_vControllerStates.push_back(*it);
    }
}

bool CRobotLaunchingLoopFunction::at_light()
{
    CPositionalEntity& light = (CPositionalEntity&) CSimulator::GetInstance().GetSpace().GetEntity("light");
    CVector3 robotPosition = m_pcFootBot->GetEmbodiedEntity().GetOriginAnchor().Position;
    CVector3 lightPosition = light.GetPosition();
    lightPosition.SetZ(0.0);

    // Return true if the robot is within half a meter of the light.
    return (robotPosition - lightPosition).Length() < 0.5;
}