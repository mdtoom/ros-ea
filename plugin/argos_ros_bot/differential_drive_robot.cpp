//
// Created by matthijs on 3-7-19.
//

#include "differential_drive_robot.h"

#include <argos3/core/utility/logging/argos_log.h>
/* Function definitions for XML parsing */
#include <argos3/core/utility/configuration/argos_configuration.h>

CControlledDifferentialDriveRobot::CControlledDifferentialDriveRobot()
    :   m_pcWheels(NULL), m_cController(NULL), leftSpeed(0), rightSpeed(0)
{}

void CControlledDifferentialDriveRobot::Init(TConfigurationNode &t_node)
{
    m_pcWheels = GetActuator<CCI_DifferentialSteeringActuator>("differential_steering");
}

void CControlledDifferentialDriveRobot::ControlStep() {

    if (m_cController == nullptr)
    {
        LOGERR << "Executing step without controller" << std::endl;
        LOGERR.Flush();
    } else {
        std::vector<Real> sensor_readings = get_sensor_readings();

        // Activate the controller.
        std::vector<Real> outputs = m_cController->activate(sensor_readings);
        set_actuators(outputs);
    }
}

void CControlledDifferentialDriveRobot::set_actuators(std::vector<Real> outputs)
{
    static const Real speed_multiplier = 50.0;
    leftSpeed =  outputs[0] * speed_multiplier;
    rightSpeed = outputs[1] * speed_multiplier;

    m_pcWheels->SetLinearVelocity(leftSpeed, rightSpeed);
}

void CControlledDifferentialDriveRobot::set_controller(CRobotController *controller) {
    m_cController = controller;
}

CRobotController *CControlledDifferentialDriveRobot::get_controller() {
    return m_cController;
}

