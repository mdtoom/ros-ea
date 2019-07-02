//
// Created by matthijs on 2-7-19.
//

#ifndef MA_EVOLUTION_SUMMED_IN_LIGHT_RANGE_FITNESS_FUNCTION_H
#define MA_EVOLUTION_SUMMED_IN_LIGHT_RANGE_FITNESS_FUNCTION_H

#include "fitness_function.h"
#include <argos3/core/utility/datatypes/datatypes.h>
#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>

/** This fitness function sums the moments that the robot is within x meter from the light. */
class CSummedInLightRangeFitnessFunction : public CFitnessFunction
{
public:
    CSummedInLightRangeFitnessFunction(CFootBotEntity& robot_entity, Real in_range_distance);

    virtual void post_step_evaluation() override;

protected:
    Real m_fInRangeDistance;

    CFootBotEntity& m_cRobotEntity;
};


#endif //MA_EVOLUTION_SUMMED_IN_LIGHT_RANGE_FITNESS_FUNCTION_H
