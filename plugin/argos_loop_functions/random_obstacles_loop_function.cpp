//
// Created by matthijs on 5-7-19.
//

#include "random_obstacles_loop_function.h"
#include "fitness_functions/absolute_distance_fitness_function.h"

#include <string>
#include <argos3/plugins/simulator/entities/box_entity.h>
#include <argos3/core/utility/math/quaternion.h>

#define  NUM_BLOCKS 4

using namespace argos;

void CRandomObstaclesLoopFunction::Init(TConfigurationNode &t_node)
{
    CGenomeRunnerLoopFunction::Init(t_node);

    // Store the start_locations.
    for (int i = 0; i < NUM_BLOCKS; i++)
    {
        CBoxEntity& cBox = dynamic_cast<CBoxEntity&>(GetSpace().GetEntity("obstacle" + std::to_string(i)));
        CVector3 position = cBox.GetEmbodiedEntity().GetOriginAnchor().Position;
        m_vObstacleLocations.emplace_back(position);
    }


    // Absolute fitness because the space dependent fitness is not equal in all scenarios now.
    delete  m_pFitnessFunction;
    m_pFitnessFunction = new CAbsoluteDistanceFitnessFunction(*m_pcFootBot, 1.0);
}

void CRandomObstaclesLoopFunction::Reset()
{
    CGenomeRunnerLoopFunction::Reset();

    for (int i = 0; i < NUM_BLOCKS; i++)
    {
        CBoxEntity& cBox = dynamic_cast<CBoxEntity&>(GetSpace().GetEntity("obstacle" + std::to_string(i)));

        Real y_offset = m_pcRNG->Uniform(CRange<Real>(-1.0, 1.0));

        // Move the light entity to the wanted position
        cBox.GetEmbodiedEntity().MoveTo(CVector3(m_vObstacleLocations[i][0], m_vObstacleLocations[i][1] + y_offset,
                m_vObstacleLocations[i][2]), CQuaternion());

    }

    // Move the obstacles to a random new location in range [-1, 1];

}

REGISTER_LOOP_FUNCTIONS(CRandomObstaclesLoopFunction, "random_obstacles_runner_loop_functions")
