//
// Created by matthijs on 18-6-19.
//

#ifndef MA_EVOLUTION_SELECTOR_STATE_MACHINE_H
#define MA_EVOLUTION_SELECTOR_STATE_MACHINE_H

#include "perceptron_network.h"
#include "state_machine_controller.h"

/**
 * This class is gives a state in a state machine controller that switches state using transitions.
 */
class CSelectorState : public CControllerState {

public:
    CSelectorState(int state_id, const CPerceptronNetwork pn, const CPerceptronNetwork selector_pn);

    virtual int next_state(const std::vector<Real> inputs) override;

private:

    CPerceptronNetwork m_cSelectorNetwork;
};


#endif //MA_EVOLUTION_SELECTOR_STATE_MACHINE_H
