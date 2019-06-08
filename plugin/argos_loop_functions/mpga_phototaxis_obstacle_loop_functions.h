#ifndef MPGA_PHOTOTAXIS_OBSTACLE_LOOP_FUNCTIONS_H
#define MPGA_PHOTOTAXIS_OBSTACLE_LOOP_FUNCTIONS_H

/* ARGoS-related headers */
#include <argos3/core/utility/math/rng.h>
#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>
#include <argos3/core/simulator/loop_functions.h>
#include <argos3/core/simulator/entity/embodied_entity.h>

#include <queue>
#include <ros/ros.h>
#include "std_srvs/Empty.h"
#include "ma_evolution/SimScore.h"
#include "geometry_msgs/Point.h"
#include "ma_evolution/Trajectory.h"
#include "ma_evolution/StateRequest.h"
#include "ma_evolution/SMGenome.h"
#include "genome_receiver.h"

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

    virtual void PreStep();

    virtual void PostStep();

    /* Calculates the performance of the robot in a trial */
    virtual Real Score();

    /** This function gets the score of the robot. */
    virtual bool GetScore(ma_evolution::SimScore::Request& request, ma_evolution::SimScore::Response& response);

    /** This function returns the latest trajectory of the robot until reset. */
    virtual bool GetTrajectory(ma_evolution::Trajectory::Request& request, ma_evolution::Trajectory::Response& response);

    /** This function returns the state history of the robot until reset. */
    virtual bool GetStateHistory(ma_evolution::StateRequest::Request& request, ma_evolution::StateRequest::Response& response);

    // We need only a single ROS node, although there are individual publishers
    // and subscribers for each instance of the class.
    static ros::NodeHandle* nodeHandle;

protected:

    /** This server returns the current score of the simulation. */
    ros::ServiceServer m_pcScoreService;
    /** This returns the list of locations of the robot from the latest reset. */
    ros::ServiceServer m_pcTrajectoryService;
    /** This service returns the state history of the current controller. */
    ros::ServiceServer m_pcStateHistoryService;
    /** This publisher publishes the score messages of the robots that have been evaluated. */
    ros::Publisher m_pcScorePublisher;

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

private:
    /** This variable stores the genomes that need to be evaluated on this controller. */
    CGenomeReceiver<ma_evolution::SMGenome> m_cGenomeReceiver;

    Real m_fFitnessPower;

    std::vector<geometry_msgs::Point> m_vLocations;
    std::vector<int> m_vControllerStates;

    int m_iExecutedSteps;
    int m_iTargetExecutedSteps;

};

#endif
