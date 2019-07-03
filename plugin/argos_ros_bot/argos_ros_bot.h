/*
 * AUTHOR: Andrew Vardy <av@mun.ca>
 *
 * Connects an ARGoS robot with a particular configuration to ROS by publishing
 * sensor values and subscribing to a desired wheel speeds topic.
 *
 */

#ifndef ARGOS_ROS_BOT_H
#define ARGOS_ROS_BOT_H

#include "differential_drive_robot.h"

#include <argos3/core/utility/datatypes/datatypes.h>
#include <argos3/plugins/robots/generic/control_interface/ci_proximity_sensor.h>
#include <argos3/plugins/robots/foot-bot/control_interface/ci_footbot_light_sensor.h>

using namespace argos;

class CArgosRosBot : public CControlledDifferentialDriveRobot {

public:

  CArgosRosBot();

  virtual void Init(TConfigurationNode& t_node);

protected:

    virtual std::vector<Real> get_sensor_readings() override;

private:

    CCI_ProximitySensor* m_pcProximity;
    CCI_FootBotLightSensor* m_pcLight;
};

#endif
