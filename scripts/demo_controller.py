#!/usr/bin/env python

"""
Demo controller for controlling an ARGoS simulated robot via argos_bridge.
The algorithm implemented here is a simple state machine designed to push
pucks to the walls.  If the proximity sensor detects an obstacle (other robot
or wall) then it moves away from it for a fixed period.
"""
import rospy
import math, random
from ma_evolution.msg import Puck
from ma_evolution.msg import PuckList
from ma_evolution.msg import Proximity
from ma_evolution.msg import ProximityList
from ma_evolution.msg import LightList
from geometry_msgs.msg import Twist


class DemoController:

    cmdVelPub = None
    puckList = None
    state = "WANDER"
    time = 0
    stateStartTime = 0
    lastTwist = None
    
    # Constants
    MAX_FORWARD_SPEED = 1
    MAX_ROTATION_SPEED = 2.5

    def __init__(self):
        self.cmdVelPub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        rospy.Subscriber('puck_list', PuckList, self.pucks_callback)
        rospy.Subscriber('light', LightList, self.light_callback)

    def pucks_callback(self, puckList):
        # All the action happens in 'prox_callback'.  Just store this most
        # list of pucks for use there.
        self.puckList = puckList

    def light_callback(self, lightList):
        print([light.value for light in lightList.lights])

        twist = Twist()
        twist.angular.z = 0.5

        self.cmdVelPub.publish(twist)



if __name__ == '__main__':

    rospy.init_node("demo_controller")
    controller = DemoController()
    rospy.spin()
