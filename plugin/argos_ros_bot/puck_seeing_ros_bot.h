//
// Created by matthijs on 3-7-19.
//

#ifndef MA_EVOLUTION_PUCK_SEEING_ROS_BOT_H
#define MA_EVOLUTION_PUCK_SEEING_ROS_BOT_H

#include "differential_drive_robot.h"
#include <argos3/plugins/robots/generic/control_interface/ci_proximity_sensor.h>
#include <argos3/plugins/robots/generic/control_interface/ci_colored_blob_omnidirectional_camera_sensor.h>


class CPuckSeeingRosBot : public CControlledDifferentialDriveRobot
{
public:
    CPuckSeeingRosBot();

    virtual void Init(TConfigurationNode& t_node);

protected:

    virtual std::vector<Real> get_sensor_readings() override;

private:

    CCI_ProximitySensor* m_pcProximity;
    CCI_ColoredBlobOmnidirectionalCameraSensor* m_pcOmniCam;
};


#endif //MA_EVOLUTION_PUCK_SEEING_ROS_BOT_H
