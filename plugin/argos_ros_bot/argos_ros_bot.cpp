/* Include the controller definition */
#include "argos_ros_bot.h"
/* Function definitions for XML parsing */
#include <argos3/core/utility/configuration/argos_configuration.h>
/* 2D vector definition */
#include <argos3/core/utility/math/vector2.h>

using namespace std;

/****************************************/
/****************************************/

CArgosRosBot::CArgosRosBot() :
  m_pcWheels(NULL),
  m_pcProximity(NULL),
  m_cController(NULL),
  leftSpeed(0),
  rightSpeed(0)//,
{ }

void CArgosRosBot::Init(TConfigurationNode& t_node)
{
    // Get sensor/actuator handles
    m_pcWheels = GetActuator<CCI_DifferentialSteeringActuator>("differential_steering");
    m_pcProximity = GetSensor<CCI_ProximitySensor>("proximity");
    m_pcLight = GetSensor<CCI_FootBotLightSensor>("footbot_light");
}


void CArgosRosBot::ControlStep() {


    if (m_cController == nullptr)
    {
        LOGERR << "Executing step without controller" << std::endl;
        LOGERR.Flush();
    } else {

        // Read the sensors.
        const std::vector<Real>& tProxReads = m_pcProximity->GetReadings();
        const CCI_FootBotLightSensor::TReadings& tLightReads = m_pcLight->GetReadings();

        // Create a vector for the sensor readings.
        std::vector<Real> sensor_readings(tProxReads.size() + tLightReads.size());

        for (size_t i = 0; i < tProxReads.size(); ++i)
        {
            sensor_readings[i] = tProxReads[i];
        }

        for (size_t i = 0; i < tLightReads.size(); ++i) {
            sensor_readings[i + tProxReads.size()] = tLightReads[i].Value;
        }

        // Activate the controller.
        std::vector<Real> outputs = m_cController->activate(sensor_readings);
        Real v = outputs[0];
        Real w = outputs[1];

        // Use the kinematics of a differential-drive robot to derive the left and right wheel speeds.
        leftSpeed = (v - HALF_BASELINE * w) / WHEEL_RADIUS;
        rightSpeed = (v + HALF_BASELINE * w) / WHEEL_RADIUS;
        m_pcWheels->SetLinearVelocity(leftSpeed, rightSpeed);
    }
}

void CArgosRosBot::set_controller(CRobotController *controller) {
    m_cController = controller;
}

CRobotController *CArgosRosBot::get_controller() {
    return m_cController;
}


REGISTER_CONTROLLER(CArgosRosBot, "argos_ros_bot_controller")
