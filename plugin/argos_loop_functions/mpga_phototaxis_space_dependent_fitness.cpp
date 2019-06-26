//
// Created by matthijs on 22-5-19.
//

#include "mpga_phototaxis_space_dependent_fitness.h"

#include <queue>
#include <string>
#include <limits>
#include <algorithm>
#include <argos3/plugins/simulator/entities/light_entity.h>

#define FITNESS_STEP_SIZE 0.25

template <class T> bool inVector(std::vector<T> vector, T element)
{
    return std::find(vector.begin(), vector.end(), element) != vector.end();
}


CMPGAPhototaxisStateDependentFitnessFunction::CMPGAPhototaxisStateDependentFitnessFunction()
{
    m_fMaxDistance = 0.0;
}


void CMPGAPhototaxisStateDependentFitnessFunction::Init(TConfigurationNode& t_node) {
    CMPGAPhototaxisObstacleLoopFunctions::Init(t_node);

    CRange<CVector3> arenaLimits = GetSpace().GetArenaLimits();
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
            MoveEntity(m_pcFootBot->GetEmbodiedEntity(), groundLocation, CQuaternion(), true))
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
            // TODO: write fitness that finds closest nodes and takes its distance as length.
            // Location sensitive hashing?
            // Fetch 4 closest locations and take minimum.

        }
    }

    m_fMaxDistance = *max_element(locationDistances.begin(), locationDistances.end());

    // Setup the fitness vector
    for (Real distance : locationDistances)
    {
        m_vLocationFitness.emplace_back(calculateFitness(distance));
    }

//    // Adds lights to every data point.
//    int count = 0;
//    for (int i = 0; i < m_vFitnessMeasureLocation.size(); i++)
//    {
//        // Add light for the view.
//        CVector3 lightLocation(m_vFitnessMeasureLocation[i].GetX(), m_vFitnessMeasureLocation[i].GetY(), m_vLocationFitness[i] * 2);
//        CLightEntity *pcLight = new CLightEntity(std::to_string(count), lightLocation, CColor::WHITE, 1.0);
//        AddEntity(*pcLight);
//        count++;
//    }
}

bool CMPGAPhototaxisStateDependentFitnessFunction::within2DBorders(CVector3 &loc, CRange<CVector3> &areaLimits)
{
    return loc[0] < areaLimits.GetMax()[0] && loc[0] > areaLimits.GetMin()[0]
           && loc[1] < areaLimits.GetMax()[1] && loc[1] > areaLimits.GetMin()[1];
}


Real CMPGAPhototaxisStateDependentFitnessFunction::CalculateStepScore()
{
    // Find the known point closest to the current point of the robot.
    CVector3 robotPosition = m_pcFootBot->GetEmbodiedEntity().GetOriginAnchor().Position;

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
    return m_vLocationFitness[min_index] + minLength;
}


/****************************************/
/****************************************/

REGISTER_LOOP_FUNCTIONS(CMPGAPhototaxisStateDependentFitnessFunction, "mpga_phototaxis_space_dependent_function")
