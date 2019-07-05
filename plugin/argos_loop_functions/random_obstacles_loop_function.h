//
// Created by matthijs on 5-7-19.
//

#ifndef MA_EVOLUTION_RANDOM_OBSTACLES_LOOP_FUNCTION_H
#define MA_EVOLUTION_RANDOM_OBSTACLES_LOOP_FUNCTION_H

#include "genome_runner_loop_function.h"

#include <vector>
#include <argos3/core/utility/math/vector3.h>

class CRandomObstaclesLoopFunction : public CGenomeRunnerLoopFunction
{

public:

    virtual void Init(TConfigurationNode& t_node) override;

    virtual void Reset() override;


private:

    /** This vector stores the initial vector locations. */
    std::vector<CVector3> m_vObstacleLocations;
};


#endif //MA_EVOLUTION_RANDOM_OBSTACLES_LOOP_FUNCTION_H
