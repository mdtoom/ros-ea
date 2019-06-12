//
// Created by matthijs on 7-6-19.
//

#ifndef MA_EVOLUTION_STATE_MACHINE_CONTROLLER_H
#define MA_EVOLUTION_STATE_MACHINE_CONTROLLER_H

#include "robot_controller.h"
#include "perceptron_network.h"

/** This class gives a template for a state in a state machine controller. Switching states should be implemented by
 * subclasses.
 */
class CControllerState {

public:
    CControllerState(int state_id, const CPerceptronNetwork pn);

    /**
     * This function activates the current state with the given inputs.
     * @return      - vector containing the outputs of the calculation.
     */
    std::vector<Real> activate(const std::vector<Real> inputs);

    /**
     * This function returns the next state of the controller. Own index is returned when no state change is made.
     * @param inputs    - The sensor inputs to the controller.
     * @return          - Index of the next state to go to.
     */
    virtual int next_state(const std::vector<Real> inputs) = 0;

    const int m_iStateId;

private:

    CPerceptronNetwork m_cPerceptronNetwork;
};


class CStateMachineController : public CRobotController {

public:

    CStateMachineController(int controller_id, int gen_id, std::vector<CControllerState*> states);
    ~CStateMachineController();

    /**
     * This function activates the state machine controller with the given inputs.
     * @return      - vector containing the outputs of the calculation.
     */
    std::vector<Real> activate(const std::vector<Real> inputs) override;

    CControllerState *current_state();

    CControllerState *get_state(int state_id);

private:
    int m_iCurrentState;

    std::vector<CControllerState*> m_vStates;
};


#endif //MA_EVOLUTION_STATE_MACHINE_CONTROLLER_H
