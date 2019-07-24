#ifndef MPGA_PHOTOTAXIS_OBSTACLE_LOOP_FUNCTIONS_H
#define MPGA_PHOTOTAXIS_OBSTACLE_LOOP_FUNCTIONS_H

/* ARGoS-related headers */
#include <argos3/core/utility/math/rng.h>
#include <argos3/core/simulator/entity/embodied_entity.h>

#include <queue>
#include "std_srvs/Empty.h"
#include "ma_evolution/SMGenome.h"
#include "genome_receiver.h"
#include "fitness_evaluating_loop_function.h"

/****************************************/
/****************************************/

using namespace argos;

class CGenomeRunnerLoopFunction: public CFitnessEvaluatingLoopFunction
{
public:

    CGenomeRunnerLoopFunction();

    virtual ~CGenomeRunnerLoopFunction();

    virtual void Init(TConfigurationNode& t_node);

    virtual void Reset();

    virtual void PreStep();

    virtual void PostStep();

protected:

    void send_score_message(Real score, CRobotController *controller);

    /** This function is called when an experiment is finished and should wrap up the iteration. */
    virtual void finish_simulation_iteration();

    /** This publisher publishes the score messages of the robots that have been evaluated. */
    ros::Publisher m_pcScorePublisher;

private:
    /** This variable stores the genomes that need to be evaluated on this controller. */
    CGenomeBuffer *m_cGenomeBuffer;

    int m_iExecutedSteps;
    int m_iTargetExecutedSteps;

    int m_iCurrentGenHash;

};

#endif
