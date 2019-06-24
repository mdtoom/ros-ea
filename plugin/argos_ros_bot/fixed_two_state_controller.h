//
// Created by matthijs on 24-6-19.
//

#ifndef MA_EVOLUTION_FIXED_TWO_STATE_CONTROLLER_H
#define MA_EVOLUTION_FIXED_TWO_STATE_CONTROLLER_H

#include "robot_controller.h"
#include "state_machine_controller.h"
#include "transition_state_machine.h"

class CFixedTwoStateController : public CRobotController {
    /** This class works on fixed two states where a transition is made when an object is within a certain distance,
     * from the obstacle.
     */

public:
    CFixedTwoStateController(int controller_id, int gen_id, const CTransitionedState &state1,
            const CTransitionedState &state2);

    std::vector<Real> activate(const std::vector<Real> inputs) override;

protected:

    bool object_in_range(const std::vector<Real> inputs);

private:

    CTransitionedState m_cState1;
    CTransitionedState m_cState2;
};


#endif //MA_EVOLUTION_FIXED_TWO_STATE_CONTROLLER_H
