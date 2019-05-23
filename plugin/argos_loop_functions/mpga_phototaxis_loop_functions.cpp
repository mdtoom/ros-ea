#include "mpga_phototaxis_loop_functions.h"
#include "../argos_ros_bot/argos_ros_bot.h"

/****************************************/
/****************************************/

CMPGAPhototaxisLoopFunctions::CMPGAPhototaxisLoopFunctions() :
   m_pcFootBot(NULL),
   m_pcRNG(NULL),
   m_bDone(false) {

}

CMPGAPhototaxisLoopFunctions::~CMPGAPhototaxisLoopFunctions()
{
    delete m_pcFootBot;
}

void CMPGAPhototaxisLoopFunctions::SetStartLocation() {

    m_vecResetLocation.Position.FromSphericalCoords(
            4.5f,                                          // distance from origin
            CRadians::PI_OVER_TWO,                         // angle with Z axis
            static_cast<Real>(1) * CRadians::PI / 12.0f // rotation around Z
    );

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

void CMPGAPhototaxisLoopFunctions::Init(TConfigurationNode& t_node) {
    /*
    * Create the random number generator
    */
    m_pcRNG = CRandom::CreateRNG("argos");

    SetStartLocation();

    // Register a reset service at the node of the simulation.
    m_pcResetService = CArgosRosBot::nodeHandle->
            advertiseService("reset", &CMPGAPhototaxisLoopFunctions::ResetRobot, this);

    // Register the get score service of the node of the simulation.
    m_pcScoreService = CArgosRosBot::nodeHandle->
            advertiseService("score", &CMPGAPhototaxisLoopFunctions::GetScore, this);

    m_pcTrajectoryService = CArgosRosBot::nodeHandle->
            advertiseService("trajectory", &CMPGAPhototaxisLoopFunctions::GetTrajectory, this);

    m_pcDoneService = CArgosRosBot::nodeHandle->advertiseService("done", &CMPGAPhototaxisLoopFunctions::IsDone, this);

    /*
    * Create the foot-bot and get a reference to its controller
    */
    m_pcFootBot = new CFootBotEntity(
      "fb",    // entity id
      "argos_ros_bot"    // controller id as set in the XML
      );
    AddEntity(*m_pcFootBot);
    Reset();
}

/****************************************/
/****************************************/

void CMPGAPhototaxisLoopFunctions::Reset() {

    /*
    * Move robot to the initial position corresponding to the current trial
    */
    if(!MoveEntity(
            m_pcFootBot->GetEmbodiedEntity(),             // move the body of the robot
            m_vecResetLocation.Position,    // to this position
            m_vecResetLocation.Orientation, // with this orientation
            false                                         // this is not a check, leave the robot there
    )) {
        LOGERR << "Can't move robot in <"
               << m_vecResetLocation.Position
               << ">, <"
               << m_vecResetLocation.Orientation
               << ">"
               << std::endl;
    }

    m_vLocations.clear();
    m_bDone = false;
}


void CMPGAPhototaxisLoopFunctions::PostStep()
{
    CVector3 robotPosition = m_pcFootBot->GetEmbodiedEntity().GetOriginAnchor().Position;

    geometry_msgs::Point point;
    point.x = robotPosition.GetX();
    point.y = robotPosition.GetY();
    point.z = robotPosition.GetZ();

    m_vLocations.emplace_back(point);
}


bool CMPGAPhototaxisLoopFunctions::ResetRobot(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response) {

    Reset();
    return true;

}

bool CMPGAPhototaxisLoopFunctions::GetScore(ma_evolution::SimScore::Request& request,
                                                    ma_evolution::SimScore::Response& response)
{
    Real score = Score();
    response.score = (float) score;
    return true;
}

bool CMPGAPhototaxisLoopFunctions::IsDone(ma_evolution::Done::Request& request, ma_evolution::Done::Response& response)
{
    response.done = m_bDone;
    return true;
}

bool CMPGAPhototaxisLoopFunctions::GetTrajectory(ma_evolution::Trajectory::Request& request,
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

Real CMPGAPhototaxisLoopFunctions::Score() {
   /* The performance is simply the distance of the robot to the origin */
   return 10.0 - m_pcFootBot->GetEmbodiedEntity().GetOriginAnchor().Position.Length();
}



/****************************************/
/****************************************/

REGISTER_LOOP_FUNCTIONS(CMPGAPhototaxisLoopFunctions, "mpga_phototaxis_loop_functions")
