//
// Created by matthijs on 22-5-19.
//

#ifndef MA_EVOLUTION_MPGA_PHOTOTAXIS_SPACE_DEPENDENT_FITNESS_H
#define MA_EVOLUTION_MPGA_PHOTOTAXIS_SPACE_DEPENDENT_FITNESS_H

#include "mpga_phototaxis_obstacle_loop_functions.h"
#include <argos3/core/utility/math/vector3.h>
#include <argos3/core/utility/math/range.h>
#include <vector>



class CMPGAPhototaxisStateDependentFitnessFunction : public CMPGAPhototaxisObstacleLoopFunctions
{
public:

    CMPGAPhototaxisStateDependentFitnessFunction();

    virtual void Init(TConfigurationNode& t_node);

protected:

    virtual Real CalculateStepScore() override;

private:


    /**
     * This function checks whether the given location is within the 2D borders of the environment.
     * @param loc           - Location, Note that z value is not taken into account.
     * @param areaLimits    - Area borders.
     * @return              - True if the location is within the 2D borders of the given area.
     */
    bool within2DBorders(CVector3 &loc, CRange<CVector3> &areaLimits);

    // These to vectors need to be of the same length and the index of a location in the fitness measure location
    // should point to the distance of that point in the other vector.
    std::vector<CVector3> m_vFitnessMeasureLocation;
    std::vector<Real> m_vLocationDistance;
};


#endif //MA_EVOLUTION_MPGA_PHOTOTAXIS_SPACE_DEPENDENT_FITNESS_H
