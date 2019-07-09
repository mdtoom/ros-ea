//
// Created by matthijs on 26-6-19.
//

#ifndef MA_EVOLUTION_ROBOT_LAUNCHING_LOOP_FUNCTION_H
#define MA_EVOLUTION_ROBOT_LAUNCHING_LOOP_FUNCTION_H

#include <ros/ros.h>
#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>
#include <argos3/core/simulator/loop_functions.h>

#include "geometry_msgs/Point.h"
#include "ma_evolution/Trajectory.h"
#include "ma_evolution/StateRequest.h"
#include "ma_evolution/AtLight.h"

using namespace argos;

/* The initial setup of a trial */
struct SInitSetup {
    CVector3 Position;
    CQuaternion Orientation;
};

class CRobotLaunchingLoopFunction : public CLoopFunctions {

public:

    CRobotLaunchingLoopFunction();

    virtual ~CRobotLaunchingLoopFunction();

    virtual void Init(TConfigurationNode& t_node);

    virtual void Reset();

    virtual void PostStep();

    // We need only a single ROS node, although there are individual publishers
    // and subscribers for each instance of the class.
    static ros::NodeHandle* nodeHandle;

protected:

    /** This function returns the latest trajectory of the robot until reset. */
    virtual bool GetTrajectory(ma_evolution::Trajectory::Request& request,
                               ma_evolution::Trajectory::Response& response);

    /** This function returns the state history of the robot until reset. */
    virtual bool GetStateHistory(ma_evolution::StateRequest::Request& request,
                                 ma_evolution::StateRequest::Response& response);

    /** This function returns the state history of the robot until reset. */
    virtual bool GetAtLight(ma_evolution::AtLight::Request& request,
                                 ma_evolution::AtLight::Response& response);

    /** This function sets the initial location of the robot. */
    virtual void SetStartLocation();

    /** This service returns the state history of the current controller. */
    ros::ServiceServer m_pcStateHistoryService;
    /** This returns the list of locations of the robot from the latest reset. */
    ros::ServiceServer m_pcTrajectoryService;
    /** This returns whether the robot is within 0.5 meter of the light. */
    ros::ServiceServer m_pcAtLightService;

    CRandom::CRNG* m_pcRNG;

    CFootBotEntity* m_pcFootBot;

    /** This function gathers the controller states from the robot. */
    void gather_controller_states();

    SInitSetup m_vecResetLocation;

private:

    std::vector<geometry_msgs::Point> m_vLocations;
    std::vector<int> m_vControllerStates;
};


#endif //MA_EVOLUTION_ROBOT_LAUNCHING_LOOP_FUNCTION_H
