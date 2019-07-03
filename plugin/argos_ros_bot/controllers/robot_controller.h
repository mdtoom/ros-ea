//
// Created by matthijs on 7-6-19.
//

#ifndef MA_EVOLUTION_ROBOT_CONTROLLER_H
#define MA_EVOLUTION_ROBOT_CONTROLLER_H

#include <vector>
#include <argos3/core/utility/datatypes/datatypes.h>

using namespace argos;

class CRobotController {
    /** This abstract class should be implemented to map the sensor inputs to actuator outputs. */

public:

    CRobotController(int id, int gen_id) : m_iID(id), m_iGenerationID(gen_id) { }

    /** This function should execute the controller. */
    virtual std::vector<Real> activate(std::vector<Real> input) = 0;

    /** This vector can be used to store the states of the robot. */
    std::vector<int> m_vStateHistory;

    int m_iID;
    int m_iGenerationID;

};


#endif //MA_EVOLUTION_ROBOT_CONTROLLER_H
