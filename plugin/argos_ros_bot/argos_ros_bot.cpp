/* Include the controller definition */
#include "argos_ros_bot.h"
/* 2D vector definition */
#include <argos3/core/utility/math/vector2.h>

using namespace std;

/****************************************/
/****************************************/

CArgosRosBot::CArgosRosBot() :

  m_pcProximity(NULL),
  m_pcLight(NULL)
{ }

void CArgosRosBot::Init(TConfigurationNode& t_node)
{
    // Get sensor/actuator handles
    CControlledDifferentialDriveRobot::Init(t_node);
    m_pcProximity = GetSensor<CCI_ProximitySensor>("proximity");
    m_pcLight = GetSensor<CCI_FootBotLightSensor>("footbot_light");
}

std::vector<Real> CArgosRosBot::get_sensor_readings()
{
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

    return sensor_readings;
}

REGISTER_CONTROLLER(CArgosRosBot, "argos_ros_bot_controller")
