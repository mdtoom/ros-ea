//
// Created by matthijs on 26-6-19.
//

#include "space_dependent_fitness_function.h"
#include <queue>
#include <argos3/core/simulator/medium/medium.h>

#define FITNESS_STEP_SIZE 0.25

using namespace argos;

template <class T> bool inVector(std::vector<T> vector, T element)
{
    return std::find(vector.begin(), vector.end(), element) != vector.end();
}

CSpaceDependentFitnessFunction::CSpaceDependentFitnessFunction(CFootBotEntity &robot_entity, Real fitness_power,
                                                               CLoopFunctions& loop_function)
    : CAbsoluteDistanceFitnessFunction(robot_entity, fitness_power)
{
    CRange<CVector3> arenaLimits = loop_function.GetSpace().GetArenaLimits();
    std::queue<CVector3> toVisitLocations;
    std::vector<CVector3> locationDepth;
    std::vector<Real> locationDistances;

    // Add the initial location to be visited.
    toVisitLocations.push(CVector3(4,2.5,0));

    while (!toVisitLocations.empty())
    {
        // Get the oldest element.
        CVector3 toEval = toVisitLocations.front();
        toVisitLocations.pop();

        // Generate required mutations.
        CVector2 loc(toEval[0], toEval[1]);
        CVector3 groundLocation(toEval[0], toEval[1], 0.0);

        // Check if not already evaluated and if location does not contain an obstacle.
        if (!inVector(m_vFitnessMeasureLocation, loc) &&
            loop_function.MoveEntity(m_cRobotEntity.GetEmbodiedEntity(), groundLocation, CQuaternion(), true))
        {
            // Put the vector in the list with evaluated genomes.
            locationDepth.emplace_back(toEval);
            m_vFitnessMeasureLocation.push_back(loc);
            locationDistances.push_back(toEval[2]);

            // Generate 4 new locations in every direction and add them to the list that needs to be evaluated.
            Real evaluators[] = {-FITNESS_STEP_SIZE, 0, FITNESS_STEP_SIZE};
            for (Real delta_x : evaluators) {
                for (Real delta_y : evaluators) {
                    CVector3 new_loc(toEval[0] + delta_x, toEval[1] + delta_y, toEval[2] + FITNESS_STEP_SIZE);
                    CVector2 new_loc_2D(new_loc[0], new_loc[1]);

                    if (within2DBorders(new_loc, arenaLimits) && !inVector(m_vFitnessMeasureLocation, new_loc_2D)) {
                        toVisitLocations.push(new_loc);
                    }
                }
            }
        }
    }

    m_fMaxDistance = *max_element(locationDistances.begin(), locationDistances.end());

    // Setup the fitness vector
    for (Real distance : locationDistances)
    {
        m_vLocationFitness.emplace_back(calculate_distance_based_fitness(distance));
    }
}

void CSpaceDependentFitnessFunction::post_step_evaluation()
{
    // Find the known point closest to the current point of the robot.
    CVector3 robotPosition = m_cRobotEntity.GetEmbodiedEntity().GetOriginAnchor().Position;

    Real minLength = std::numeric_limits<float>::max();
    int min_index = 0;
    for (int i = 0; i < m_vFitnessMeasureLocation.size(); i++)
    {
        CVector2 compVec =  m_vFitnessMeasureLocation[i] - CVector2(robotPosition[0], robotPosition[1]);
        Real compLength = compVec.Length();

        if (compLength < minLength)
        {
            minLength = compLength;
            min_index = i;
        }
    }

    m_fScore += m_vLocationFitness[min_index] + minLength;
}


bool CSpaceDependentFitnessFunction::within2DBorders(CVector3 &loc, CRange<CVector3> &areaLimits)
{
    return loc[0] < areaLimits.GetMax()[0] && loc[0] > areaLimits.GetMin()[0]
           && loc[1] < areaLimits.GetMax()[1] && loc[1] > areaLimits.GetMin()[1];
}