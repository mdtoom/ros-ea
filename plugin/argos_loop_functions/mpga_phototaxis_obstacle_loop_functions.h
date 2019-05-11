#ifndef MPGA_PHOTOTAXIS_OBSTACLE_LOOP_FUNCTIONS_H
#define MPGA_PHOTOTAXIS_OBSTACLE_LOOP_FUNCTIONS_H

/* ARGoS-related headers */
#include <argos3/core/utility/math/rng.h>
#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>

#include "mpga_phototaxis_loop_functions.h"

#include "ros/ros.h"
#include "std_srvs/Empty.h"
#include "ma_evolution/SimScore.h"

/****************************************/
/****************************************/

using namespace argos;

class CMPGAPhototaxisObstacleLoopFunctions : public CMPGAPhototaxisLoopFunctions
{

public:

   CMPGAPhototaxisObstacleLoopFunctions();
   virtual ~CMPGAPhototaxisObstacleLoopFunctions() = default;

   virtual void Init(TConfigurationNode& t_node);
   virtual void Reset();

   /** This function resets the robot to its original position. */
   virtual bool ResetRobot(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);

   virtual bool GetScore(ma_evolution::SimScore::Request& request, ma_evolution::SimScore::Response& response);

   /* Calculates the performance of the robot in a trial */
   virtual Real Score();



};

#endif
