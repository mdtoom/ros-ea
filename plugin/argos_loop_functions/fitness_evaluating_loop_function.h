//
// Created by matthijs on 26-6-19.
//

#ifndef MA_EVOLUTION_FITNESS_EVALUATING_LOOP_FUNCTION_H
#define MA_EVOLUTION_FITNESS_EVALUATING_LOOP_FUNCTION_H

#include "robot_launching_loop_function.h"

#include <ros/ros.h>
#include "fitness_functions/fitness_function.h"
#include "ma_evolution/SimScore.h"

class CFitnessEvaluatingLoopFunction : public CRobotLaunchingLoopFunction {
public:

    CFitnessEvaluatingLoopFunction();

    virtual ~CFitnessEvaluatingLoopFunction();

    virtual void Init(TConfigurationNode& t_node);

    virtual void Reset();

    virtual void PostStep();

protected:

    /** This server returns the current score of the simulation. */
    ros::ServiceServer m_pcScoreService;

    /** This function gets the score of the robot. */
    virtual bool GetScore(ma_evolution::SimScore::Request& request, ma_evolution::SimScore::Response& response);

    CFitnessFunction *m_pFitnessFunction;

};


#endif //MA_EVOLUTION_FITNESS_EVALUATING_LOOP_FUNCTION_H
