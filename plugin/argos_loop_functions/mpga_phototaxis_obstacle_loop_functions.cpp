#include "mpga_phototaxis_obstacle_loop_functions.h"
#include "../argos_ros_bot/argos_ros_bot.h"
#include <argos3/core/simulator/simulator.h>
#include <argos3/core/simulator/entity/positional_entity.h>
#include <argos3/core/utility/math/vector3.h>
#include <cmath>

/****************************************/
/****************************************/

using namespace argos;

#define FITNESS_POWER_DEFAULT 3            // This define is used to indicate the shape of the fitness function.

const Real maxDistance = 11.0;      // Constant used to indicate the max distance from the robot to the light.

CMPGAPhototaxisObstacleLoopFunctions::CMPGAPhototaxisObstacleLoopFunctions() :
    m_fScore(0), m_fMaxDistance(maxDistance), m_pcFootBot(NULL), m_pcRNG(NULL),
    m_bDone(false), m_fFitnessPower(FITNESS_POWER_DEFAULT)
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

    // Register a reset service at the node of the simulation.
    m_pcResetService = CArgosRosBot::nodeHandle->
            advertiseService("reset", &CMPGAPhototaxisObstacleLoopFunctions::ResetRobot, this);

    // Register the get score service of the node of the simulation.
    m_pcScoreService = CArgosRosBot::nodeHandle->
            advertiseService("score", &CMPGAPhototaxisObstacleLoopFunctions::GetScore, this);

    m_pcTrajectoryService = CArgosRosBot::nodeHandle->
            advertiseService("trajectory", &CMPGAPhototaxisObstacleLoopFunctions::GetTrajectory, this);

    m_pcDoneService = CArgosRosBot::nodeHandle->advertiseService("done",
            &CMPGAPhototaxisObstacleLoopFunctions::IsDone, this);

    // Create the foot-bot and get a reference to its controller
    m_pcFootBot = new CFootBotEntity("fb", "argos_ros_bot");
    AddEntity(*m_pcFootBot);
    Reset();
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
    m_bDone = false;
    m_fScore = 0;
}

bool CMPGAPhototaxisObstacleLoopFunctions::ResetRobot(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response) {

    Reset();
    return true;

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

bool CMPGAPhototaxisObstacleLoopFunctions::IsDone(ma_evolution::Done::Request& request, ma_evolution::Done::Response& response)
{
    response.done = m_bDone;
    return true;
}


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

void CMPGAPhototaxisObstacleLoopFunctions::PostStep() {

    // Add the current robot location to the list of visited locations.
    CVector3 robotPosition = m_pcFootBot->GetEmbodiedEntity().GetOriginAnchor().Position;
    geometry_msgs::Point point;
    point.x = robotPosition.GetX();
    point.y = robotPosition.GetY();
    point.z = robotPosition.GetZ();
    m_vLocations.emplace_back(point);

    if (!m_bDone)
    {
        // After a collision do not update the score anymore.
        if (m_pcFootBot->GetEmbodiedEntity().IsCollidingWithSomething())
        {
            m_bDone = true;
        }
        m_fScore += CalculateStepScore();
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
