//
// Created by matthijs on 26-6-19.
//

#ifndef MA_EVOLUTION_FITNESS_FUNCTIONS_H
#define MA_EVOLUTION_FITNESS_FUNCTIONS_H

#include <argos3/core/utility/datatypes/datatypes.h>

using namespace argos;

/** This abstract class defines what a fitness function should look like, important is to implement the
 * post_step_evaluation() function, that should implement how the fitness changes.
 */
class CFitnessFunction {

public:

    CFitnessFunction();

    /** this function should be called after each step and it should update the fitness as required. */
    virtual void post_step_evaluation() = 0;

    Real get_fitness();

    /** Reset the fitness calculation. */
    virtual void reset();

protected:

    Real m_fScore;

};

#endif //MA_EVOLUTION_FITNESS_FUNCTIONS_H
