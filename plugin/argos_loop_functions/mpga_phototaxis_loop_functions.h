#ifndef MPGA_PHOTOTAXIS_LOOP_FUNCTIONS_H
#define MPGA_PHOTOTAXIS_LOOP_FUNCTIONS_H

/* ARGoS-related headers */
#include <argos3/core/utility/math/rng.h>
#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>

#include "ros/ros.h"
#include "std_srvs/Empty.h"
#include "ma_evolution/SimScore.h"

#include <argos3/core/simulator/loop_functions.h>

/****************************************/
/****************************************/

using namespace argos;

class CMPGAPhototaxisLoopFunctions : public CLoopFunctions {

public:

    CMPGAPhototaxisLoopFunctions();
    virtual ~CMPGAPhototaxisLoopFunctions() = default;

    virtual void Init(TConfigurationNode& t_node);
    virtual void Reset();

    /** This function resets the robot to its original position. */
    virtual bool ResetRobot(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);

    virtual bool GetScore(ma_evolution::SimScore::Request& request, ma_evolution::SimScore::Response& response);

    /* Calculates the performance of the robot in a trial */
    virtual Real Score();

protected:

    /** This server keeps the messages for the reset service coming. */
    ros::ServiceServer m_pcResetService;
    /** This server returns the current score of the simulation. */
    ros::ServiceServer m_pcScoreService;

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
