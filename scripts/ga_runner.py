#!/usr/bin/env python
import os
import time
from threading import Condition

import neat
import rospy
from message_parsing import encode_neat_genome
from examples.experiment_template import SingleExperiment
from ma_evolution.msg import NEATGenome, Score


class ROSRobotExperiment(SingleExperiment):
    """ This class evaluates the genomes by sending them to a ROS node, which evaluates them."""

    def __init__(self, learning_config, exp_runner, num_generations, genome_encoder_func,
                 exp_name='', num_trails=1, base_directory=''):
        """ Note the additional parameter genome_encoder_func, which encodes a genome for sending. """

        SingleExperiment.__init__(self, learning_config, exp_runner, num_generations, exp_name, num_trails, base_directory)
        self.genome_encoder_func = genome_encoder_func

        # Initialise the variables needed to communicate with the others trough ROS.
        self.retrieved_scores = {}      # This variable is used to keep the retrieved scores of the ROS node.
        self.genome_publisher = rospy.Publisher('genome_topic', NEATGenome, queue_size=100)
        self.condition_lock = Condition()
        rospy.init_node('evolutionary_algorithm', anonymous=True)
        rospy.Subscriber('score_topic', Score, self.score_callback)

    def eval_genomes(self, genomes, config):

        start_time = time.time()

        self.retrieved_scores = {}      # reset the list with retrieved scores.

        # Publish the genomes to the genome topic.
        for genome_id, genome in genomes:

            ros_encoded_genome = self.genome_encoder_func(genome)
            self.genome_publisher.publish(ros_encoded_genome)
            # time.sleep(1)

        print('published all information.')

        # Wait until all scores came back.
        self.condition_lock.acquire()
        while len(self.retrieved_scores) != len(genomes):

            print(len(self.retrieved_scores))
            self.condition_lock.wait(1.0)
            print('stuck in loop, waiting for others.')

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
        self.condition_lock.acquire(True)
        self.retrieved_scores[data.key] = data.score
        self.condition_lock.release()


experiment_name = 'NEAT'
num_steps = 150
num_generations = 100
num_runs = 5
num_trials = 1


if __name__ == '__main__':

    config_location = 'config-feedforward'

    # Create learning configuration.
    local_dir = os.path.dirname(__file__)
    config_path = os.path.join(local_dir, config_location)
    config = neat.Config(neat.DefaultGenome, neat.DefaultReproduction,
                         neat.DefaultSpeciesSet, neat.DefaultStagnation,
                         config_path)

    # Create and run experiment.

    try:
        experiment = ROSRobotExperiment(config, '', num_generations, encode_neat_genome, experiment_name)

        for i in range(num_runs):
            experiment.run(experiment_name + str(i))

    except rospy.ROSInterruptException:
        pass