// ROS Stuff #include "ros/ros.h"
#include "ma_evolution/Puck.h"
#include "ma_evolution/PuckList.h"
#include "ma_evolution/Proximity.h"
#include "ma_evolution/ProximityList.h"
#include "ma_evolution/Light.h"
#include "ma_evolution/LightList.h"
#include "beginner_tutorials/AddTwoInts.h"

/* Include the controller definition */
#include "argos_ros_bot.h"
/* Function definitions for XML parsing */
#include <argos3/core/utility/configuration/argos_configuration.h>
/* 2D vector definition */
#include <argos3/core/utility/math/vector2.h>

#include <iostream>
#include <sstream>

#include <ros/callback_queue.h>

using namespace std;
using namespace ma_evolution;

// Initialize ROS node.  There will be only one ROS node no matter how many robots are created in
// ARGoS.  However, we will have one instance of the CArgosRosBot class for each ARGoS robot.
ros::NodeHandle* initROS() {
  int argc = 0;
  char *argv = (char *) "";
  ros::init(argc, &argv, "argos_bridge");
  return new ros::NodeHandle();
}

ros::NodeHandle* CArgosRosBot::nodeHandle = initROS();

/****************************************/
/****************************************/

CArgosRosBot::CArgosRosBot() :
  m_pcWheels(NULL),
  m_pcProximity(NULL),
  m_pcOmniCam(NULL),
  stopWithoutSubscriberCount(10),
  stepsSinceCallback(0),
  leftSpeed(0),
  rightSpeed(0)//,
{
}

void CArgosRosBot::Init(TConfigurationNode& t_node) {
  // Create the topics to publish
  stringstream puckListTopic, proximityTopic, lightTopic;
  puckListTopic << "/puck_list";
  proximityTopic << "/proximity";
  lightTopic << "/light";

  puckListPub = nodeHandle->advertise<PuckList>(puckListTopic.str(), 1);
  proximityPub = nodeHandle->advertise<ProximityList>(proximityTopic.str(), 1);
  lightPub = nodeHandle->advertise<LightList>(lightTopic.str(), 1);

  // Create the subscribers
  stringstream cmdVelTopic;//, gripperTopic;
  cmdVelTopic << "/cmd_vel";
  cmdVelSub = nodeHandle->subscribe(cmdVelTopic.str(), 1, &CArgosRosBot::cmdVelCallback, this);

  // Get sensor/actuator handles
  m_pcWheels = GetActuator<CCI_DifferentialSteeringActuator>("differential_steering");
  m_pcProximity = GetSensor<CCI_FootBotProximitySensor>("footbot_proximity");
  m_pcOmniCam = GetSensor<CCI_ColoredBlobOmnidirectionalCameraSensor>("colored_blob_omnidirectional_camera");
  m_pcLight = GetSensor<CCI_FootBotLightSensor>("footbot_light");

  /*
   * Parse the configuration file
   *
   * The user defines this part. Here, the algorithm accepts three
   * parameters and it's nice to put them in the config file so we don't
   * have to recompile if we want to try other settings.
   */
  GetNodeAttributeOrDefault(t_node, "stopWithoutSubscriberCount", stopWithoutSubscriberCount, stopWithoutSubscriberCount);
}

// Compares pucks for sorting purposes.  We sort by angle.
bool puckComparator(Puck a, Puck b) {
  return a.angle < b.angle;
}

void CArgosRosBot::PublishPucks() {

  const CCI_ColoredBlobOmnidirectionalCameraSensor::SReadings& camReads = m_pcOmniCam->GetReadings();
  PuckList puckList;
  puckList.n = camReads.BlobList.size();
  for (size_t i = 0; i < puckList.n; ++i) {
    Puck puck;
    puck.type = (camReads.BlobList[i]->Color == CColor::RED);
    puck.range = camReads.BlobList[i]->Distance;
    // Make the angle of the puck in the range [-PI, PI].  This is useful for
    // tasks such as homing in on a puck using a simple controller based on
    // the sign of this angle.
    puck.angle = camReads.BlobList[i]->Angle.SignedNormalize().GetValue();
    puckList.pucks.push_back(puck);
  }

  // Sort the puck list by angle.  This is useful for the purposes of extracting meaning from
  // the local puck configuration (e.g. fitting a lines to the detected pucks).
  sort(puckList.pucks.begin(), puckList.pucks.end(), puckComparator);

  puckListPub.publish(puckList);
}

void CArgosRosBot::PublishProximity() {
  /* Get readings from proximity sensor */
  const CCI_FootBotProximitySensor::TReadings& tProxReads = m_pcProximity->GetReadings();
  ProximityList proxList;
  proxList.n = tProxReads.size();
  for (size_t i = 0; i < proxList.n; ++i) {
    Proximity prox;
    prox.value = tProxReads[i].Value;
    prox.angle = tProxReads[i].Angle.GetValue();
    proxList.proximities.push_back(prox);

  //cout << GetId() << ": value: " << prox.value << ": angle: " << prox.angle << endl;
  }

  proximityPub.publish(proxList);
}

void CArgosRosBot::PublishLight() {
    /* Get readings from light sensor */
    const CCI_FootBotLightSensor::TReadings& tLightReads = m_pcLight->GetReadings();

    LightList lightList;
    lightList.n = tLightReads.size();
    for (size_t i = 0; i < lightList.n; ++i) {
        Light light;
        light.value = tLightReads[i].Value;
        light.angle = tLightReads[i].Angle.GetValue();
        lightList.lights.push_back(light);

        //cout << GetId() << ": value: " << prox.value << ": angle: " << prox.angle << endl;
    }

    lightPub.publish(lightList);
}


void CArgosRosBot::ControlStep() {

//  PublishPucks();
  PublishProximity();
  PublishLight();

  // Wait for any callbacks to be called.
  ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(0.1));

  // If we haven't heard from the subscriber in a while, set the speed to zero.
  if (stepsSinceCallback > stopWithoutSubscriberCount) {
    leftSpeed = 0;
    rightSpeed = 0;
  } else {
    stepsSinceCallback++;
  }

  m_pcWheels->SetLinearVelocity(leftSpeed, rightSpeed);
}

void CArgosRosBot::cmdVelCallback(const geometry_msgs::Twist& twist) {

  Real v = twist.linear.x;  // Forward speed
  Real w = twist.angular.z; // Rotational speed

  // Use the kinematics of a differential-drive robot to derive the left
  // and right wheel speeds.
  leftSpeed = (v - HALF_BASELINE * w) / WHEEL_RADIUS;
  rightSpeed = (v + HALF_BASELINE * w) / WHEEL_RADIUS;

  stepsSinceCallback = 0;
}

REGISTER_CONTROLLER(CArgosRosBot, "argos_ros_bot_controller")
