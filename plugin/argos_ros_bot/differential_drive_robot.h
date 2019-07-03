//
// Created by matthijs on 3-7-19.
//

#ifndef MA_EVOLUTION_DIFFERENTIAL_DRIVE_ROBOT_H
#define MA_EVOLUTION_DIFFERENTIAL_DRIVE_ROBOT_H

#include <argos3/core/utility/datatypes/datatypes.h>
#include <argos3/core/control_interface/ci_controller.h>
#include <argos3/plugins/robots/generic/control_interface/ci_differential_steering_actuator.h>
#include <vector>

#include "robot_controller.h"

using namespace argos;

class CControlledDifferentialDriveRobot : public CCI_Controller
{
public:

    CControlledDifferentialDriveRobot();

    virtual ~CControlledDifferentialDriveRobot() = default;

    virtual void Init(TConfigurationNode& t_node);

    virtual void ControlStep();

    virtual void Reset() {}

    virtual void Destroy() {}

    /**
     * This function sets the controller of the robot to the given controller.
     * @param controller      - Controller of the robot.
     */
    void set_controller(CRobotController *controller);
    CRobotController * get_controller();

protected:

    virtual std::vector<Real> get_sensor_readings() = 0;
    virtual void set_actuators(std::vector<Real> outputs);

private:

    CCI_DifferentialSteeringActuator* m_pcWheels;

    // The following constant values were copied from the argos source tree from
    // the file src/plugins/robots/foot-bot/simulator/footbot_entity.cpp
    static constexpr Real HALF_BASELINE = 0.07f; // Half the distance between wheels
    static constexpr Real WHEEL_RADIUS = 0.029112741f;


    // Most recent left and right wheel speeds.
    Real leftSpeed, rightSpeed;

    CRobotController *m_cController;

};


#endif //MA_EVOLUTION_DIFFERENTIAL_DRIVE_ROBOT_H
