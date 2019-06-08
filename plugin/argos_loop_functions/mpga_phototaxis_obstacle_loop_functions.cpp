#include "mpga_phototaxis_obstacle_loop_functions.h"
#include "../argos_ros_bot/argos_ros_bot.h"
#include <argos3/core/simulator/simulator.h>
#include <argos3/core/simulator/entity/positional_entity.h>
#include <argos3/core/utility/math/vector3.h>
#include <cmath>
#include <iostream>
#include <sstream>

/****************************************/
/****************************************/

using namespace argos;

#define FITNESS_POWER_DEFAULT 3            // This define is used to indicate the shape of the fitness function.

const Real maxDistance = 11.0;      // Constant used to indicate the max distance from the robot to the light.


// Initialize ROS node.  There will be only one ROS node no matter how many robots are created in
// ARGoS.  However, we will have one instance of the CArgosRosBot class for each ARGoS robot.
ros::NodeHandle* initROS() {
    int argc = 0;
    char *argv = (char *) "";
    ros::init(argc, &argv, "argos_bridge");
    return new ros::NodeHandle();
}

ros::NodeHandle* CMPGAPhototaxisObstacleLoopFunctions::nodeHandle = initROS();

CMPGAPhototaxisObstacleLoopFunctions::CMPGAPhototaxisObstacleLoopFunctions() :
    m_fScore(0), m_fMaxDistance(maxDistance), m_pcFootBot(NULL), m_pcRNG(NULL), m_fFitnessPower(FITNESS_POWER_DEFAULT)
{ }

/****************************************/
/****************************************/

CMPGAPhototaxisObstacleLoopFunctions::~CMPGAPhototaxisObstacleLoopFunctions()
{
    delete m_pcFootBot;
}

/****************************************/
/****************************************/

void CMPGAPhototaxisObstacleLoopFunctions::Init(TConfigurationNode& t_node)
{

    /* Create the random number generator. */
    m_pcRNG = CRandom::CreateRNG("argos");

    // Set the power attribute which is used for deciding the power of the fitness calculation.
    GetNodeAttributeOrDefault(t_node, "fitness_power", m_fFitnessPower, m_fFitnessPower);

    SetStartLocation();


    // Register the get score service of the node of the simulation.
    m_pcScoreService = nodeHandle->
            advertiseService("score", &CMPGAPhototaxisObstacleLoopFunctions::GetScore, this);

    m_pcTrajectoryService = nodeHandle->
            advertiseService("trajectory", &CMPGAPhototaxisObstacleLoopFunctions::GetTrajectory, this);

    m_pcStateHistoryService = nodeHandle->
            advertiseService("states_request", &CMPGAPhototaxisObstacleLoopFunctions::GetStateHistory, this);


    // Create the subscribers
    std::stringstream genome_topic_str;
    genome_topic_str << nodeHandle->getNamespace() << "/genome_topic";
    m_pcGenomeSub = nodeHandle->subscribe(genome_topic_str.str(), 1000,
            &CMPGAPhototaxisObstacleLoopFunctions::receive_genome, this);

    // Create the foot-bot and get a reference to its controller
    m_pcFootBot = new CFootBotEntity("fb", "argos_ros_bot");
    AddEntity(*m_pcFootBot);
    Reset();
}

CRobotController *decodeROSMsg(msg)
{
    // TODO:: implement.
    return nullptr
}

void CMPGAPhototaxisObstacleLoopFunctions::receive_genome(const ma_evolution::SMGenome& msg) {
    CRobotController *controller = decodeROSMsg(msg);
    m_qControllerQueue.put(controller);

    LOG << "Received genome" << std::endl;
    LOG.Flush();
}

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
    m_fScore = 0;
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

bool CMPGAPhototaxisObstacleLoopFunctions::GetTrajectory(ma_evolution::Trajectory::Request& request,
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

bool CMPGAPhototaxisObstacleLoopFunctions::GetStateHistory(ma_evolution::StateRequest::Request &request,
                                                           ma_evolution::StateRequest::Response &response) {

    for(std::vector<int>::iterator it = m_vControllerStates.begin(); it != m_vControllerStates.end(); ++it)
    {
        response.StateSequence.emplace_back(*it);
    }

    return true;
}

/****************************************/
/****************************************/

void CMPGAPhototaxisObstacleLoopFunctions::PreStep()
{
    CArgosRosBot &controller = (CArgosRosBot&) m_pcFootBot->GetControllableEntity().GetController();

    ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(0.1));

    // Wait until a genome is send.
    while(m_qGenomeQueue.empty() && c)
    {
        ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(0.1));
        LOG << "No genome available, waiting." << std::endl;
    }


    LOG << "Executed pre step" << std::endl;
    LOG.Flush();
}

/****************************************/
/****************************************/

void CMPGAPhototaxisObstacleLoopFunctions::PostStep() {

    // Add the current robot location to the list of visited locations.
    CVector3 robotPosition = m_pcFootBot->GetEmbodiedEntity().GetOriginAnchor().Position;
    geometry_msgs::Point point;
    point.x = robotPosition.GetX();
    point.y = robotPosition.GetY();
    point.z = robotPosition.GetZ();
    m_vLocations.emplace_back(point);

    m_fScore += CalculateStepScore();

    if (m_pcFootBot->GetEmbodiedEntity().IsCollidingWithSomething())
    {
        //TODO: remove controller and set variables.
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
    return pow(normalizedDistance, m_fFitnessPower);
}


/****************************************/
/****************************************/

REGISTER_LOOP_FUNCTIONS(CMPGAPhototaxisObstacleLoopFunctions, "mpga_phototaxis_obstacle_loop_functions")
