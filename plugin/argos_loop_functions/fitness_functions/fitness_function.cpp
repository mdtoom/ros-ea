//
// Created by matthijs on 26-6-19.
//

#include "fitness_function.h"
#include <cmath>
#include <argos3/core/simulator/simulator.h>
#include <argos3/core/utility/math/vector3.h>
#include <argos3/plugins/simulator/entities/light_entity.h>

CFitnessFunction::CFitnessFunction() : m_fScore(0.0)
{}

Real CFitnessFunction::get_fitness()
{
    return m_fScore;
}

void CFitnessFunction::reset()
{
    m_fScore = 0.0;
}