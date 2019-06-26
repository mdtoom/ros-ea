//
// Created by matthijs on 26-6-19.
//

#ifndef MA_EVOLUTION_SPACE_DEPENDENT_FITNESS_FUNCTION_H
#define MA_EVOLUTION_SPACE_DEPENDENT_FITNESS_FUNCTION_H

#include "absolute_distance_fitness_function.h"
#include <argos3/core/utility/datatypes/datatypes.h>
#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>
#include <argos3/core/simulator/loop_functions.h>

using namespace argos;

class CSpaceDependentFitnessFunction : public CAbsoluteDistanceFitnessFunction
{
public:
    CSpaceDependentFitnessFunction(CFootBotEntity& robot_entity, Real fitness_power, CLoopFunctions& loop_function);

    virtual void post_step_evaluation() override;

private:

    /**
     * This function checks whether the given location is within the 2D borders of the environment.
     * @param loc           - Location, Note that z value is not taken into account.
     * @param areaLimits    - Area borders.
     * @return              - True if the location is within the 2D borders of the given area.
     */
    bool within2DBorders(CVector3 &loc, CRange<CVector3> &areaLimits);

    // These to vectors need to be of the same length and the index of a location in the fitness measure location
    // should point to the fitness of that point in the other vector.
    std::vector<CVector2> m_vFitnessMeasureLocation;
    std::vector<Real> m_vLocationFitness;

};



#endif //MA_EVOLUTION_SPACE_DEPENDENT_FITNESS_FUNCTION_H
