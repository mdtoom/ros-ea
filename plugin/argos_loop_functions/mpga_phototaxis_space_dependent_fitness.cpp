//
// Created by matthijs on 22-5-19.
//

#include "mpga_phototaxis_space_dependent_fitness.h"

#include <queue>
#include <string>
#include <argos3/plugins/simulator/entities/light_entity.h>

#define FITNESS_STEP_SIZE 0.25

CMPGAPhototaxisStateDependentFitnessFunction::CMPGAPhototaxisStateDependentFitnessFunction()
{ }


void CMPGAPhototaxisStateDependentFitnessFunction::Init(TConfigurationNode& t_node) {
    CMPGAPhototaxisObstacleLoopFunctions::Init(t_node);


    CRange<CVector3> arenaLimits = GetSpace().GetArenaLimits();
    std::vector<CVector2> visitedLocations;
    std::queue<CVector3> toVisitLocations;
    std::vector<CVector3> locationDepth;

    toVisitLocations.push(CVector3(4,2.5,0));

    int count = 0;

    while (!toVisitLocations.empty())
    {
        CVector3 toEval = toVisitLocations.front();
        toVisitLocations.pop();
        CVector2 loc(toEval[0], toEval[1]);
        CVector3 groundLocation(toEval[0], toEval[1], 0.0);

        if (std::find(visitedLocations.begin(), visitedLocations.end(), loc) != visitedLocations.end() ||
            !MoveEntity(m_pcFootBot->GetEmbodiedEntity(), groundLocation, m_vecResetLocation.Orientation,true))
        {
            continue;
        }

        // Put the vector in the list with evaluated genomes.
        locationDepth.emplace_back(toEval);
        visitedLocations.emplace_back(loc);

        // Add light for the view.
        CVector3 lightLocation(toEval[0], toEval[1], toEval[2]);
        CLightEntity *pcLight = new CLightEntity(std::to_string(count), lightLocation, CColor::WHITE, 1.0);
        AddEntity(*pcLight);

        // Generate 4 new locations in every direction and add them to the list that needs to be evaluated.
        Real evaluators[] = {-FITNESS_STEP_SIZE, 0, FITNESS_STEP_SIZE};
        for (Real delta_x : evaluators)
        {
            for (Real delta_y : evaluators)
            {
                CVector3 new_loc(toEval[0] + delta_x, toEval[1] + delta_y, toEval[2] + FITNESS_STEP_SIZE);
                CVector2 new_loc_2D(new_loc[0], new_loc[1]);

                if (within2DBorders(new_loc, arenaLimits) &&
                    std::find(visitedLocations.begin(), visitedLocations.end(), new_loc_2D) == visitedLocations.end())
                {
                    toVisitLocations.push(new_loc);
                }
            }
        }
        count++;
        // TODO: write fitness that finds closest nodes and takes its distance as length.
        // Location sensitive hashing?
        // Fetch 4 closest locations and take minimum.
    }
}

bool CMPGAPhototaxisStateDependentFitnessFunction::within2DBorders(CVector3 &loc, CRange<CVector3> &areaLimits)
{
    return loc[0] < areaLimits.GetMax()[0] && loc[0] > areaLimits.GetMin()[0]
           && loc[1] < areaLimits.GetMax()[1] && loc[1] > areaLimits.GetMin()[1];
}


Real CMPGAPhototaxisStateDependentFitnessFunction::CalculateStepScore()
{
    LOG << "Using new function" << std::endl;
    LOG.Flush();
//    CPositionalEntity& light = (CPositionalEntity&) CSimulator::GetInstance().GetSpace().GetEntity("light");
//    CVector3 robotPosition = m_pcFootBot->GetEmbodiedEntity().GetOriginAnchor().Position;
//    CVector3 lightPosition = light.GetPosition();
//    lightPosition.SetZ(robotPosition.GetZ());
//    CVector3 differenceVector = robotPosition - lightPosition;
//
//    // Get points for closeness to light, as long as the robot did not collide.
//
//    Real normalizedDistance = 1.0 - differenceVector.Length() / maxDistance;
//
//    return pow(normalizedDistance, FITNESS_POWER);
    return 0.0;
}


/****************************************/
/****************************************/

REGISTER_LOOP_FUNCTIONS(CMPGAPhototaxisStateDependentFitnessFunction, "mpga_phototaxis_space_dependent_function")
