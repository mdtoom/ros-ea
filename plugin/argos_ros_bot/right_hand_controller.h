//
// Created by matthijs on 25-6-19.
//

#ifndef MA_EVOLUTION_RIGHT_HAND_BOT_H
#define MA_EVOLUTION_RIGHT_HAND_BOT_H

#include "robot_controller.h"


class CRightHandBot : public CRobotController
{
public:

    CRightHandBot();

    ~CRightHandBot() = default;
    /** This function should execute the controller. */
    virtual std::vector<Real> activate(std::vector<Real> input);

private:

    int m_iState;

};


#endif //MA_EVOLUTION_RIGHT_HAND_BOT_H
