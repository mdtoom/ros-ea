//
// Created by matthijs on 23-7-19.
//

#ifndef MA_EVOLUTION_COME_AND_GO_FITNESS_FUNCTION_H
#define MA_EVOLUTION_COME_AND_GO_FITNESS_FUNCTION_H

#include "fitness_function.h"
#include <argos3/core/utility/datatypes/datatypes.h>
#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>

class CComeAndGoFitnessFunction : public CFitnessFunction {

public:
    CComeAndGoFitnessFunction(CFootBotEntity& robot_entity);

    virtual void post_step_evaluation() override;

    virtual void reset() override;

protected:

    CFootBotEntity& m_cRobotEntity;

private:

    /** This variable becomes true if the robot is close to the light. */
    bool m_bEncounteredLight;
};


#endif //MA_EVOLUTION_COME_AND_GO_FITNESS_FUNCTION_H
