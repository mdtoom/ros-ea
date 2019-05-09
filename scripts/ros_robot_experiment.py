#!/usr/bin/env python
import time
from random import randint
from threading import Condition

import rospy
from examples.experiment_template import SingleExperiment
from ma_evolution.msg import Score


class SimulationCommunicator:
    """ This class takes communication with one simulation runner."""

    def __init__(self, namespace, message_type, score_callback):
        print(namespace)
        self.genome_publisher = rospy.Publisher(namespace + 'genome_topic', message_type, queue_size=100)
        rospy.Subscriber(namespace + 'score_topic', Score, score_callback)

    def publish_genome(self, enc_genome):
        self.genome_publisher.publish(enc_genome)


class ROSRobotExperiment(SingleExperiment):
    """ This class evaluates the genomes by sending them to a ROS node, which evaluates them."""

    def __init__(self, learning_config, exp_runner, num_generations, genome_encoder,
                 exp_name='', num_trails=1, base_directory=''):
        """ Note the additional parameter genome_encoder, which encodes a genome for sending. """

        SingleExperiment.__init__(self, learning_config, exp_runner, num_generations, exp_name, num_trails,
                                  base_directory)
        self.genome_encoder = genome_encoder

        # Initialise the variables needed to communicate with the others trough ROS.
        self.retrieved_scores = {}  # This variable is used to keep the retrieved scores of the ROS node.
        self.condition_lock = Condition()
        self.gen_hash = 0
        rospy.init_node('evolutionary_algorithm', anonymous=True)

        # Find the running simulation nodes
        search_string = 'score_topic'
        score_topics = [topic[0] for topic in rospy.get_published_topics() if search_string in topic[0]]
        name_spaces = [topic.replace(search_string, '') for topic in score_topics]
        self.sim_controllers = [SimulationCommunicator(ns, genome_encoder.get_message_type(), self.score_callback)
                                for ns in name_spaces]
        time.sleep(1)       # Sleep is required for initialisation

    def send_genomes(self, genomes):
        """ This function resends the genomes, which have not yet replied. """

        controller_count = 0
        for genome_id, genome in genomes:

            assert genome_id == genome.key  # Safety check, so both can be used as keys.
            if genome_id not in self.retrieved_scores:

                #print('Sending genome {0} to simulation {1}'.format(genome_id, controller_count))
                ros_encoded_genome = self.genome_encoder.encode(genome, self.gen_hash)
                self.sim_controllers[controller_count].publish_genome(ros_encoded_genome)
                controller_count = (controller_count + 1) % len(self.sim_controllers)

    def eval_genomes(self, genomes, config):
        start_time = time.time()

        self.gen_hash = randint(0, 1000000)  # Used to ensure that messages from a previous run are ignored.
        self.retrieved_scores = {}  # reset the list with retrieved scores.
        self.send_genomes(genomes)

        # Wait until all scores came back.
        self.condition_lock.acquire()

        previous_received = 0
        while len(self.retrieved_scores) != len(genomes):

            self.condition_lock.wait(10.0)

            if previous_received == len(self.retrieved_scores):
                print('Did not received new scores, got %s so far', previous_received)
                print('Resending missing genomes')
                self.send_genomes(genomes)

            previous_received = len(self.retrieved_scores)

            if rospy.is_shutdown():
                raise rospy.ROSInterruptException()

        self.condition_lock.release()

        # Set the scores to the genomes.
        for genome_id, genome in genomes:
            genome.fitness = self.retrieved_scores[genome_id]

        end_time = time.time()
        time_diff = end_time - start_time
        avg_time = time_diff / len(genomes)

        print("generation total_runtime: %s seconds, avg_runtime: %s seconds" % (time_diff, avg_time))

    def score_callback(self, data):
        # Put the retrieved score in a list and notify (trough the condition) that a new score has arrived.
        #print('retrieved score {0} of gen {1}'.format(data.key, data.gen_hash))

        if self.gen_hash == data.gen_hash:  # Ignore messages from a different generation.
            self.retrieved_scores[data.key] = data.score
            self.condition_lock.acquire(True)
            self.condition_lock.notify()
            self.condition_lock.release()
