#include "mpga_phototaxis_loop_functions.h"

/****************************************/
/****************************************/

CMPGAPhototaxisLoopFunctions::CMPGAPhototaxisLoopFunctions() :
   m_pcFootBot(NULL),
   m_pcRNG(NULL) {}

/****************************************/
/****************************************/

CMPGAPhototaxisLoopFunctions::~CMPGAPhototaxisLoopFunctions() {
}

/****************************************/
/****************************************/

void CMPGAPhototaxisLoopFunctions::Init(TConfigurationNode& t_node) {
   /*
    * Create the random number generator
    */
   m_pcRNG = CRandom::CreateRNG("argos");

   /*
    * Create the foot-bot and get a reference to its controller
    */
   m_pcFootBot = new CFootBotEntity(
      "fb",    // entity id
      "argos_ros_bot"    // controller id as set in the XML
      );
   AddEntity(*m_pcFootBot);

   /*
    * Create the initial setup for each trial
    * The robot is placed 4.5 meters away from the light
    * (which is in the origin) at angles
    * { PI/12, 2*PI/12, 3*PI/12, 4*PI/12, 5*PI/12 }
    * wrt to the world reference.
    * Also, the rotation of the robot is chosen at random
    * from a uniform distribution.
    */

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

   LOG << "Running function" << std::endl;

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
}

/****************************************/
/****************************************/

Real CMPGAPhototaxisLoopFunctions::Score() {
   /* The performance is simply the distance of the robot to the origin */
   return m_pcFootBot->GetEmbodiedEntity().GetOriginAnchor().Position.Length();
}

/****************************************/
/****************************************/

REGISTER_LOOP_FUNCTIONS(CMPGAPhototaxisLoopFunctions, "mpga_phototaxis_loop_functions")
