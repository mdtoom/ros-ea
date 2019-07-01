//
// Created by matthijs on 1-7-19.
//

#ifndef MA_EVOLUTION_LIGHT_RANDOM_INIT_RUNNER_H
#define MA_EVOLUTION_LIGHT_RANDOM_INIT_RUNNER_H

#include "genome_runner_loop_function.h"

class CLightRandomInitLoopFunction : public CGenomeRunnerLoopFunction {

    virtual void Init(TConfigurationNode& t_node);

    virtual void Reset();

private:

    CVector3 m_vLightPosition;

};


#endif //MA_EVOLUTION_LIGHT_RANDOM_INIT_RUNNER_H
