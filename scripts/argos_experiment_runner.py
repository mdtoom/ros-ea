#!/usr/bin/env python

import os
from Queue import Queue

import neat
import rospy
from examples.experiment_functions import FeedForwardNetworkController
from std_srvs.srv import Empty
from ma_evolution.msg import Score, NEATGenome
from ma_evolution.msg import Puck
from ma_evolution.msg import PuckList
from ma_evolution.msg import Proximity
from ma_evolution.msg import ProximityList
from ma_evolution.msg import LightList
from geometry_msgs.msg import Twist

from message_parsing import decode_neat_genome


class ArgosExperimentRunner:
    """ This class runs an experiment by connecting to argos evaluating a genome. """

    proximityList = None
    time_steps = 0
    current_controller = None

    def __init__(self, num_steps, controller_class, config):
        self.num_steps = num_steps
        self.controller_class = controller_class
        self.config = config
        self.genome_queue = Queue()

        # Publisher and subscribers of genome sender.
        self.scorePublisher = rospy.Publisher('score_topic', Score, queue_size=10)
        rospy.init_node('executioner', anonymous=True)
        rospy.Subscriber('genome_topic', NEATGenome, self.genome_callback)

        # Publisher and subscribers of comminication with ArGos simulator.
        self.cmdVelPub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        rospy.Subscriber('proximity', ProximityList, self.proximity_callback)
        rospy.Subscriber('light', LightList, self.light_callback)

    def genome_callback(self, data):
        print('Genome data')
        self.genome_queue.put(data)

    def proximity_callback(self, proximityList):
        self.proximityList = proximityList

    def light_callback(self, lightList):

        print('Executed control loop')
        twist = Twist()
        twist.linear.x = 0  # Forward speed
        twist.angular.z = 0  # Rotational speed

        if self.current_controller is not None:

            print('Evaluating Genome')
            # Extract the sensor data.
            proximities = [proximity.value for proximity in self.proximityList.proximities]
            lights = [light.value for light in lightList.lights]
            observation = proximities + lights

            print(len(observation))

            # Execute the controller given the data.
            actions = self.current_controller.step(observation)
            assert len(actions) == 2

            twist.linear.x = actions[0]
            twist.angular.z = actions[1]

            self.time_steps += 1

            # if the simulation has run for the required number of time steps, reset the current controller.
            if self.time_steps >= self.num_steps:
                print("Executed a full simulation.")
                self.current_controller = None

        # If no genome is running, then set a new available genome to run.
        elif not self.genome_queue.empty():
            encoded_genome = self.genome_queue.get()
            genome = decode_neat_genome(encoded_genome)
            self.current_controller = self.controller_class()
            self.current_controller.reset(genome, self.config)
            self.time_steps = 0

            self.reset_simulation()

        self.cmdVelPub.publish(twist)

    @staticmethod
    def reset_simulation():

        # execute the reset service of the simulation.
        rospy.wait_for_service('reset')
        reset_simulator = rospy.ServiceProxy('reset', Empty)
        reset_simulator()


if __name__ == '__main__':
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.

    print('Setup started')
    config_location = 'config-feedforward'

    local_dir = os.path.dirname(__file__)
    config_path = os.path.join(local_dir, config_location)
    config = neat.Config(neat.DefaultGenome, neat.DefaultReproduction,
                         neat.DefaultSpeciesSet, neat.DefaultStagnation,
                         config_path)
    print('Config Loaded')

    exp_runner = ArgosExperimentRunner(150, FeedForwardNetworkController, config)
    print('Runner created')

    print('Setup finished')

    rospy.spin()
