//
// Created by matthijs on 26-6-19.
//

#ifndef MA_EVOLUTION_ABSOLUTE_DISTANCE_FITNESS_FUNCTION_H
#define MA_EVOLUTION_ABSOLUTE_DISTANCE_FITNESS_FUNCTION_H

#include "fitness_function.h"
#include <argos3/core/utility/datatypes/datatypes.h>
#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>

using namespace argos;

class CAbsoluteDistanceFitnessFunction : public CFitnessFunction
{
public:
    CAbsoluteDistanceFitnessFunction(CFootBotEntity& robot_entity, Real fitness_power);

    virtual void post_step_evaluation() override;

protected:

    Real calculate_distance_based_fitness(Real distance);

    Real m_fMaxDistance;

    CFootBotEntity& m_cRobotEntity;

    Real m_fFitnessPower;

};


#endif //MA_EVOLUTION_ABSOLUTE_DISTANCE_FITNESS_FUNCTION_H
