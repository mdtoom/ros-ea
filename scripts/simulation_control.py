from time import sleep

import rospy
import roslaunch
from examples.experiment_functions import FFControllerFactory, SMControllerFactory
from ma_evolution.msg import SimulationReport
from ma_evolution.srv import Trajectory, StateRequest, SimScore, AtLight
from neat.state_machine_network import StateMachineNetwork
from neat.state_selector_network import StateSelectorNetwork
from std_srvs.srv import Empty

from message_parsing import NEATROSEncoder, SMROSEncoder, SMSROSEncoder

controller_selector = {
    'feed-forward': (FFControllerFactory(), NEATROSEncoder),
    'state-machine': (SMControllerFactory(StateMachineNetwork), SMROSEncoder),
    'state-selector': (SMControllerFactory(StateSelectorNetwork), SMSROSEncoder)
}


class SimulationCommunicator:
    """ This class takes communication with one simulation runner."""

    def __init__(self, namespace, message_type, condition_lock):
        print(namespace)
        self.namespace = namespace
        self.condition_lock = condition_lock
        self.genome_publisher = rospy.Publisher(self.namespace + 'genome_topic', message_type, queue_size=100)
        self.retrieved_scores = {}
        self.expected_gen_hash = 0
        rospy.Subscriber(self.namespace + 'simreport_topic', SimulationReport, self.callback)

    def reset(self):
        self.expected_gen_hash = 0
        self.retrieved_scores = {}

    def publish_genome(self, enc_genome):
        self.expected_gen_hash = enc_genome.header.gen_hash
        self.genome_publisher.publish(enc_genome)

    def get_score(self):
        """ This function returns the last score of the simulator this runner controls."""
        rospy.wait_for_service(self.namespace + 'score')
        score_service = rospy.ServiceProxy(self.namespace + 'score', SimScore)
        return score_service().score

    def get_trajectory(self):
        rospy.wait_for_service(self.namespace + 'trajectory')
        trajectory_service = rospy.ServiceProxy(self.namespace + 'trajectory', Trajectory)
        return trajectory_service().Locations

    def get_states(self):
        rospy.wait_for_service(self.namespace + 'states_request')
        state_service = rospy.ServiceProxy(self.namespace + 'states_request', StateRequest)
        return state_service().StateSequence

    def get_at_light(self):
        rospy.wait_for_service(self.namespace + 'at_light')
        at_light_service = rospy.ServiceProxy(self.namespace + 'at_light', AtLight)
        return at_light_service().atLight

    def callback(self, data):
        # Put the retrieved score in a list and notify (trough the condition) that a new score has arrived.

        # Ignore messages from a different generation or that have already been received before.
        if self.expected_gen_hash == data.header.gen_hash and data.header.key not in self.retrieved_scores:
            self.retrieved_scores[data.header.key] = data.score
            self.condition_lock.acquire(True)
            self.condition_lock.notify()
            self.condition_lock.release()

    def change_controller(self, controller_nm):
        """ This function changes the controller of the communicator to the controller given in the argument."""
        rospy.set_param(self.namespace + 'controller', controller_nm)
        rospy.wait_for_service(self.namespace + 'update_params')
        service = rospy.ServiceProxy(self.namespace + 'update_params', Empty)
        service()

    def run_genome(self, encoded_genome):
        self.reset()

        self.condition_lock.acquire()
        self.publish_genome(encoded_genome)

        while not self.condition_lock.wait(1.0):
            if len(self.retrieved_scores) == 1:
                break

            if rospy.is_shutdown():
                exit(1)
        self.condition_lock.release()


class SimulationController:
    """ This class launches and communicates with all activate simulation runners."""

    def __init__(self, pkg_nm, launch_file, controller_nm):

        try:
            self.genome_encoder = controller_selector[controller_nm][1]
        except KeyError:
            print('Controller type not known.')

        launch_args = ['controller:=' + controller_nm]
        arg_list = [pkg_nm, launch_file] + launch_args
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        arg_list_param = roslaunch.rlutil.resolve_launch_arguments(arg_list)
        self.launch = roslaunch.parent.ROSLaunchParent(uuid, [(arg_list_param[0], launch_args)])

    def start_simulators(self):
        """ This function starts the simulators."""
        self.launch.start()

        sleep(2)
        rospy.loginfo("started Simulators")

    def stop_simulators(self):
        """ This function stops the simulators and make this object unusable."""
        self.launch.shutdown()
