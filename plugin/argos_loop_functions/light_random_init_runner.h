//
// Created by matthijs on 1-7-19.
//

#ifndef MA_EVOLUTION_LIGHT_RANDOM_INIT_RUNNER_H
#define MA_EVOLUTION_LIGHT_RANDOM_INIT_RUNNER_H

#include "genome_runner_loop_function.h"

class CLightRandomInitLoopFunction : public CGenomeRunnerLoopFunction {

public:
    CLightRandomInitLoopFunction();

    ~CLightRandomInitLoopFunction() = default;

    virtual void Init(TConfigurationNode& t_node);

    virtual void Reset();

    virtual void finish_simulation_iteration();

private:

    CVector3 m_vLightPosition;

    int m_iNumTrials;

    int m_iMaxNumTrials;

    Real m_fCurrentControllerScore;

};


#endif //MA_EVOLUTION_LIGHT_RANDOM_INIT_RUNNER_H
