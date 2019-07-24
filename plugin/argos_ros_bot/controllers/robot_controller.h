//
// Created by matthijs on 7-6-19.
//

#ifndef MA_EVOLUTION_ROBOT_CONTROLLER_H
#define MA_EVOLUTION_ROBOT_CONTROLLER_H

#include <vector>
#include <argos3/core/utility/datatypes/datatypes.h>
#include "ma_evolution/GenomeHeader.h"

using namespace argos;

struct ControllerHeader {
    int identifier;
    int gen_hash;
    int generation;
};


class CRobotController {
    /** This abstract class should be implemented to map the sensor inputs to actuator outputs. */

public:

    CRobotController(const ControllerHeader header) : m_sHeader(header) { }

    /** This function should execute the controller. */
    virtual std::vector<Real> activate(std::vector<Real> input) = 0;

    /** This vector can be used to store the states of the robot. */
    std::vector<int> m_vStateHistory;

    const ControllerHeader m_sHeader;
};


#endif //MA_EVOLUTION_ROBOT_CONTROLLER_H
