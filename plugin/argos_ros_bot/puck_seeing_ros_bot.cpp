//
// Created by matthijs on 3-7-19.
//

#include "puck_seeing_ros_bot.h"

#include <argos3/core/utility/math/angles.h>
#include <argos3/core/utility/math/general.h>

#define NUM_PUCK_READINGS 1

std::vector<Real> CPuckSeeingRosBot::get_sensor_readings()
{
    const std::vector<Real>& tProxReads = m_pcProximity->GetReadings();
    const CCI_ColoredBlobOmnidirectionalCameraSensor::SReadings& camReads = m_pcOmniCam->GetReadings();

    // Create a vector for the sensor readings.
    std::vector<Real> sensor_readings(tProxReads.size() + NUM_PUCK_READINGS * 2);

    for (size_t i = 0; i < tProxReads.size(); ++i)
    {
        sensor_readings[i] = tProxReads[i];
    }

    for (size_t i = 0; i < NUM_PUCK_READINGS; ++i) {

        if (i < camReads.BlobList.size())
        {

#ifdef DEBUG
            LOG << "puck at:" << camReads.BlobList[i]->Distance << " : "
                << camReads.BlobList[i]->Angle.SignedNormalize().GetValue() << std::endl;
#endif
            // Get the distance.
            Real inverted_distance = Exp(-2.0 * camReads.BlobList[i]->Distance);
            // Get the angle and normalize it in [0 . 1]
            Real normalized_angle = (camReads.BlobList[i]->Angle.SignedNormalize() + CRadians::PI) / CRadians::TWO_PI;
            sensor_readings.push_back(inverted_distance);
            sensor_readings.push_back(normalized_angle);
        } else {
            sensor_readings.push_back(0.0);
            sensor_readings.push_back(0.0);
        }
    }
}

REGISTER_CONTROLLER(CPuckSeeingRosBot, "puck_seeing_robot_controller")
