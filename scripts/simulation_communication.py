import rospy
from ma_evolution.msg import Score
from ma_evolution.srv import SimScore
from ma_evolution.srv import Trajectory
from std_srvs.srv import Empty


class SimulationCommunicator:
    """ This class takes communication with one simulation runner."""

    def __init__(self, namespace, message_type, condition_lock):
        print(namespace)
        self.namespace = namespace
        self.condition_lock = condition_lock
        self.genome_publisher = rospy.Publisher(self.namespace + 'genome_topic', message_type, queue_size=100)
        self.retrieved_scores = {}
        self.gen_hash = 0
        rospy.Subscriber(self.namespace + 'score_topic', Score, self.callback)

    def reset(self):
        self.gen_hash = 0
        self.retrieved_scores = {}

    def publish_genome(self, enc_genome):
        self.gen_hash = enc_genome.gen_hash
        self.genome_publisher.publish(enc_genome)

    def get_score(self):
        """ This function returns the last score of the simulator this runner controls."""
        rospy.wait_for_service(self.namespace + 'score')
        score_service = rospy.ServiceProxy(self.namespace + 'score', SimScore)
        return score_service()

    def get_trajectory(self):
        rospy.wait_for_service(self.namespace + 'trajectory')
        trajectory_service = rospy.ServiceProxy(self.namespace + 'trajectory', Trajectory)
        return trajectory_service()

    def callback(self, data):
        # Put the retrieved score in a list and notify (trough the condition) that a new score has arrived.
        # print('retrieved score {0} of gen {1}'.format(data.key, data.gen_hash))

        if self.gen_hash == data.gen_hash:  # Ignore messages from a different generation.
            self.retrieved_scores[data.key] = data.score
            self.condition_lock.acquire(True)
            self.condition_lock.notify()
            self.condition_lock.release()

    def change_controller(self, controller_nm):
        """ This function changes the controller of the communicator to the controller given in the argument."""
        rospy.set_param(self.namespace + 'controller', controller_nm)
        rospy.wait_for_service(self.namespace + 'update_params')
        service = rospy.ServiceProxy(self.namespace + 'update_params', Empty)
        service()
