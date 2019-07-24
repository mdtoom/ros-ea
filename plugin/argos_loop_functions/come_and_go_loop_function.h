//
// Created by matthijs on 23-7-19.
//

#ifndef MA_EVOLUTION_COME_AND_GO_LOOP_FUNCTION_H
#define MA_EVOLUTION_COME_AND_GO_LOOP_FUNCTION_H

#include "genome_runner_loop_function.h"

class CComeAndGoLoopFunction : public CGenomeRunnerLoopFunction
{
    virtual void Init(TConfigurationNode& t_node) override;

    virtual void Reset() override;

};


#endif //MA_EVOLUTION_COME_AND_GO_LOOP_FUNCTION_H
