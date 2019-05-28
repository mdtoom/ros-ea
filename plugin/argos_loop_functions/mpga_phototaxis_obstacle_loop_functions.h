#ifndef MPGA_PHOTOTAXIS_OBSTACLE_LOOP_FUNCTIONS_H
#define MPGA_PHOTOTAXIS_OBSTACLE_LOOP_FUNCTIONS_H

/* ARGoS-related headers */
#include <argos3/core/utility/math/rng.h>
#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>
#include <argos3/core/simulator/loop_functions.h>
#include <argos3/core/simulator/entity/embodied_entity.h>

#include "ros/ros.h"
#include "std_srvs/Empty.h"
#include "ma_evolution/SimScore.h"
#include "geometry_msgs/Point.h"
#include "ma_evolution/Trajectory.h"
#include "ma_evolution/Done.h"

/****************************************/
/****************************************/

using namespace argos;

/* The initial setup of a trial */
struct SInitSetup {
    CVector3 Position;
    CQuaternion Orientation;
};


class CMPGAPhototaxisObstacleLoopFunctions : public CLoopFunctions
{

public:

    CMPGAPhototaxisObstacleLoopFunctions();

    virtual ~CMPGAPhototaxisObstacleLoopFunctions();

    virtual void Init(TConfigurationNode& t_node);

    virtual void Reset();

    virtual void SetStartLocation();

    virtual void PostStep();

    /* Calculates the performance of the robot in a trial */
    virtual Real Score();


    /** This function resets the robot to its original position. */
    virtual bool ResetRobot(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);

    /** This function gets the score of the robot. */
    virtual bool GetScore(ma_evolution::SimScore::Request& request, ma_evolution::SimScore::Response& response);

    /** Returns true if the robot is done. */
    virtual bool IsDone(ma_evolution::Done::Request& request, ma_evolution::Done::Response& response);

    /** This function returns the latest trajectory of the robot until reset. */
    virtual bool GetTrajectory(ma_evolution::Trajectory::Request& request, ma_evolution::Trajectory::Response& response);


protected:

    /** This server keeps the messages for the reset service coming. */
    ros::ServiceServer m_pcResetService;
    /** This server returns the current score of the simulation. */
    ros::ServiceServer m_pcScoreService;
    /** This returns the list of locations of the robot from the latest reset. */
    ros::ServiceServer m_pcTrajectoryService;
    /** This returns the list of locations of the robot from the latest reset. */
    ros::ServiceServer m_pcDoneService;

    /**
     * Calculate the fitness based on a distance to the object.
     * @param distance      - distance to object.
     * @return              - Fitness based on distance.
     */
    virtual Real calculateFitness(Real distance);

    virtual Real CalculateStepScore();

    Real m_fScore;

    Real m_fMaxDistance;

    SInitSetup m_vecResetLocation;

    CFootBotEntity* m_pcFootBot;

    CRandom::CRNG* m_pcRNG;

    bool m_bDone;

private:

    Real m_fFitnessPower;

    std::vector<geometry_msgs::Point> m_vLocations;
};

#endif
