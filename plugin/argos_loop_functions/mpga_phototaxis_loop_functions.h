#ifndef MPGA_PHOTOTAXIS_LOOP_FUNCTIONS_H
#define MPGA_PHOTOTAXIS_LOOP_FUNCTIONS_H

/* ARGoS-related headers */
#include <argos3/core/utility/math/rng.h>
#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>

#include "mpga_loop_functions.h"

/****************************************/
/****************************************/

using namespace argos;

class CMPGAPhototaxisLoopFunctions : public CMPGALoopFunctions {

public:

   CMPGAPhototaxisLoopFunctions();
   virtual ~CMPGAPhototaxisLoopFunctions();

   virtual void Init(TConfigurationNode& t_node);
   virtual void Reset();

   /* Calculates the performance of the robot in a trial */
   virtual Real Score();

private:

   /* The initial setup of a trial */
   struct SInitSetup {
      CVector3 Position;
      CQuaternion Orientation;
   };

   SInitSetup m_vecResetLocation;
   CFootBotEntity* m_pcFootBot;
   CRandom::CRNG* m_pcRNG;


};

#endif
