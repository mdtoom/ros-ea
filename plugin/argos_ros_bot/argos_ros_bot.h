/*
 * AUTHOR: Andrew Vardy <av@mun.ca>
 *
 * Connects an ARGoS robot with a particular configuration to ROS by publishing
 * sensor values and subscribing to a desired wheel speeds topic.
 *
 */

#ifndef ARGOS_ROS_BOT_H
#define ARGOS_ROS_BOT_H

#include <argos3/core/control_interface/ci_controller.h>
#include <argos3/plugins/robots/generic/control_interface/ci_differential_steering_actuator.h>
#include <argos3/plugins/robots/generic/control_interface/ci_proximity_sensor.h>
#include <argos3/plugins/robots/foot-bot/control_interface/ci_footbot_light_sensor.h>
#include <argos3/plugins/robots/generic/control_interface/ci_colored_blob_omnidirectional_camera_sensor.h>
#include <ros/ros.h>
#include "robot_controller.h"

using namespace argos;

class CArgosRosBot : public CCI_Controller {

public:

  CArgosRosBot();
  virtual ~CArgosRosBot() {}

  /*
   * This function initializes the controller.
   * The 't_node' variable points to the <parameters> section in the XML
   * file in the <controllers><footbot_ccw_wander_controller> section.
   */
  virtual void Init(TConfigurationNode& t_node);

  /*
   * This function is called once every time step.
   * The length of the time step is set in the XML file.
   */
  virtual void ControlStep();

  /*
   * This function resets the controller to its state right after the
   * Init().
   * It is called when you press the reset button in the GUI.
   * In this example controller there is no need for resetting anything,
   * so the function could have been omitted. It's here just for
   * completeness.
   */
  virtual void Reset() {}

  /*
   * Called to cleanup what done by Init() when the experiment finishes.
   * In this example controller there is no need for clean anything up,
   * so the function could have been omitted. It's here just for
   * completeness.
   */
  virtual void Destroy() {}

  /**
   * This function sets the controller of the robot to the given controller.
   * @param controller      - Controller of the robot.
   */
  void set_controller(CRobotController *controller);
  CRobotController * get_controller();

private:

  CCI_DifferentialSteeringActuator* m_pcWheels;
  CCI_ProximitySensor* m_pcProximity;
  CCI_FootBotLightSensor* m_pcLight;

  // The following constant values were copied from the argos source tree from
  // the file src/plugins/robots/foot-bot/simulator/footbot_entity.cpp
  static constexpr Real HALF_BASELINE = 0.07f; // Half the distance between wheels
  static constexpr Real WHEEL_RADIUS = 0.029112741f;


  // Most recent left and right wheel speeds.
  Real leftSpeed, rightSpeed;

  CRobotController *m_cController;

};

#endif
