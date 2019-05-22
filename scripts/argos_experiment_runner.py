#!/usr/bin/env python

import sys
from Queue import Queue

import rospy
from examples.experiment_functions import SMControllerFactory, FFControllerFactory
from neat.activations import ActivationFunctionSet
from neat.aggregations import AggregationFunctionSet
from neat.state_machine_network import StateMachineNetwork
from neat.state_selector_network import StateSelectorNetwork
from std_srvs.srv import Empty
from ma_evolution.msg import Score
from ma_evolution.msg import ProximityList
from ma_evolution.msg import LightList
from geometry_msgs.msg import Twist
from ma_evolution.srv import SimScore
from ma_evolution.srv import Done

from message_parsing import NEATROSEncoder, SMROSEncoder, SMSROSEncoder

DONE_CHECK_INTERVAL = 20


class ArgosExperimentRunner:
    """ This class runs an experiment by connecting to argos evaluating a genome. """

    controller_selector = {
        'feed-forward': (FFControllerFactory(), NEATROSEncoder),
        'state-machine': (SMControllerFactory(StateMachineNetwork), SMROSEncoder),
        'state-selector': (SMControllerFactory(StateSelectorNetwork), SMSROSEncoder)
    }

    def __init__(self, num_steps, controller_nm, num_inputs, num_outputs, clear_queue_on_new_gen=True):

        self.proximityList = None
        self.time_steps = 0
        self.current_controller = None
        self.current_genome = None
        self.genome_sub = None
        self.latest_gen_hash = 0
        self.clear_queue_on_new_gen = clear_queue_on_new_gen
        self.genome_queue = Queue()

        # These parameters are modifiable using the param server and the service update_params.
        self.num_steps = 0
        self.controller_factory = None
        self.decoder = None
        self.config = None

        # Publisher and subscribers of genome sender.
        self.scorePublisher = rospy.Publisher('score_topic', Score, queue_size=10)
        rospy.init_node('executioner', anonymous=True)
        rospy.Service('update_params', Empty, self.param_update)

        # Set the initial parameters in the server.
        rospy.set_param('controller', controller_nm)
        rospy.set_param('num_inputs', num_inputs)
        rospy.set_param('num_outputs', num_outputs)
        rospy.set_param('num_steps', num_steps)
        rospy.set_param('clear_queue_on_new_gen', clear_queue_on_new_gen)
        self.param_update(None)             # Note that this function has a considerable amount of initialization

        # Publisher and subscribers of comminication with ArGos simulator.
        self.cmdVelPub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        rospy.Subscriber('proximity', ProximityList, self.proximity_callback)
        rospy.Subscriber('light', LightList, self.light_callback)

    def genome_callback(self, data):

        # If we are currently executing a different generation, reset queue and current genome.
        if self.clear_queue_on_new_gen and self.latest_gen_hash != data.gen_hash:
            rospy.loginfo('Started with new generation, clearing genome queue.')

            self.latest_gen_hash = data.gen_hash

            with self.genome_queue.mutex:
                self.genome_queue.queue.clear()

        self.genome_queue.put(data)

    def proximity_callback(self, proximityList):
        self.proximityList = proximityList

    def light_callback(self, lightList):

        twist = Twist()
        twist.linear.x = 0  # Forward speed
        twist.angular.z = 0  # Rotational speed

        if self.current_controller is not None:

            # Extract the sensor data.
            proximities = [proximity.value for proximity in self.proximityList.proximities]
            lights = [light.value for light in lightList.lights]
            observation = proximities + lights

            # Execute the controller given the data.
            actions = self.current_controller.step(observation)
            assert len(actions) == 2

            twist.linear.x = actions[0]
            twist.angular.z = actions[1]

            self.time_steps += 1

            # if the simulation has run for the required number of time steps, reset the current controller.
            if self.time_steps >= self.num_steps:
                self.finish_simulation()
            elif self.time_steps % DONE_CHECK_INTERVAL == 0 and self.is_done():
                # Also finish when the simulation indicates it is done.
                self.finish_simulation()

        # If no genome is running, then set a new available genome to run.
        elif not self.genome_queue.empty():
            encoded_genome = self.genome_queue.get()
            self.set_new_controller(encoded_genome)

        self.cmdVelPub.publish(twist)

    def finish_simulation(self):
        score = self.get_score()
        self.publish_score(self.current_genome.key, self.current_genome.gen_hash, score)

        self.current_controller = None

    def set_new_controller(self, enc_genome):
        genome = self.decoder.decode(enc_genome)
        self.current_controller = self.controller_factory.generate(genome, self.config)
        self.current_genome = enc_genome
        self.time_steps = 0

        self.reset_simulation()

    def param_update(self, _):
        """ This function updates the runner with the current parameters."""

        # Update controller type
        try:
            controller_nm = rospy.get_param('controller')
            self.controller_factory, self.decoder = self.controller_selector[controller_nm]
            rospy.loginfo('Going for {0}'.format(self.controller_factory))
        except KeyError:
            rospy.logerr('Exiting because invalid controller type provided')
            exit(1)

        self.num_steps = rospy.get_param('num_steps')
        self.clear_queue_on_new_gen = rospy.get_param('clear_queue_on_new_gen')

        num_inputs = rospy.get_param('num_inputs')
        num_outputs = rospy.get_param('num_outputs')
        self.config = StandInGeneralConfig(num_inputs, num_outputs)

        # Resubscribe to genome topic since, message type now changes.
        if self.genome_sub is not None:
            self.genome_sub.unregister()
        self.genome_sub = rospy.Subscriber('genome_topic', self.decoder.get_message_type(), self.genome_callback)

        return []

    @staticmethod
    def reset_simulation():

        # execute the reset service of the simulation.
        rospy.wait_for_service('reset')
        reset_simulator = rospy.ServiceProxy('reset', Empty)
        reset_simulator()

    @staticmethod
    def get_score():
        """ This function gets the score from the simulation."""
        rospy.wait_for_service('score')
        get_score = rospy.ServiceProxy('score', SimScore)
        msg = get_score()
        return msg.score

    @staticmethod
    def is_done():
        rospy.wait_for_service('done')
        is_done = rospy.ServiceProxy('done', Done)
        msg = is_done()
        return msg.done

    def publish_score(self, key, gen_hash, score):
        """ This function publishes the obtained score. """
        score_message = Score(key, gen_hash, score)
        self.scorePublisher.publish(score_message)


class StandInConfig:
    """ This class represents a simplified config satisfactory to both the NEAT and SM genome,
    to create and run genomes """

    def __init__(self, num_inputs, num_outputs):

        # Create full set of available activation functions.
        self.activation_defs = ActivationFunctionSet()
        # ditto for aggregation functions - name difference for backward compatibility
        self.aggregation_function_defs = AggregationFunctionSet()
        self.aggregation_defs = self.aggregation_function_defs

        # By convention, input pins have negative keys, and the output
        # pins have keys 0,1,...
        self.input_keys = [-i - 1 for i in range(num_inputs)]
        self.output_keys = [i for i in range(num_outputs)]


class StandInGeneralConfig:
    """ This class represents a general config required for compatibility with NEAT library."""
    def __init__(self, num_inputs, num_outputs):
        self.genome_config = StandInConfig(num_inputs, num_outputs)


if __name__ == '__main__':
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.

    my_argv = rospy.myargv(argv=sys.argv)

    if len(my_argv) != 5:
        rospy.logfatal('usage: argos_experiment_runner.py controller num_steps num_inputs num_outputs')
        rospy.logfatal('but provided: ' + str(my_argv))
    else:

        rospy.loginfo('Setup started')

        controller_nm = my_argv[1]
        num_steps = int(my_argv[2])
        num_inputs = int(my_argv[3])
        num_outputs = int(my_argv[4])

        config = StandInGeneralConfig(num_inputs, num_outputs)
        exp_runner = ArgosExperimentRunner(num_steps, controller_nm, num_inputs, num_outputs)

        rospy.loginfo('Setup finished')

        rospy.spin()
