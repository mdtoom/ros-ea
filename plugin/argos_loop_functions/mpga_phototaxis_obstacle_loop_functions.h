#ifndef MPGA_PHOTOTAXIS_OBSTACLE_LOOP_FUNCTIONS_H
#define MPGA_PHOTOTAXIS_OBSTACLE_LOOP_FUNCTIONS_H

/* ARGoS-related headers */
#include <argos3/core/utility/math/rng.h>
#include <argos3/core/simulator/entity/embodied_entity.h>

#include <queue>
#include "std_srvs/Empty.h"
#include "ma_evolution/SimScore.h"
#include "ma_evolution/SMGenome.h"
#include "genome_receiver.h"
#include "robot_launching_loop_function.h"

/****************************************/
/****************************************/

using namespace argos;

class CMPGAPhototaxisObstacleLoopFunctions : public CRobotLaunchingLoopFunction
{

public:

    CMPGAPhototaxisObstacleLoopFunctions();

    virtual ~CMPGAPhototaxisObstacleLoopFunctions();

    virtual void Init(TConfigurationNode& t_node);

    virtual void Reset();

    virtual void PreStep();

    virtual void PostStep();

    /* Calculates the performance of the robot in a trial */
    virtual Real Score();

    /** This function gets the score of the robot. */
    virtual bool GetScore(ma_evolution::SimScore::Request& request, ma_evolution::SimScore::Response& response);

protected:

    /** This server returns the current score of the simulation. */
    ros::ServiceServer m_pcScoreService;
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

private:
    /** This variable stores the genomes that need to be evaluated on this controller. */
    CGenomeBuffer *m_cGenomeBuffer;

    Real m_fFitnessPower;


    int m_iExecutedSteps;
    int m_iTargetExecutedSteps;

    int m_iCurrentGenHash;

};

#endif
