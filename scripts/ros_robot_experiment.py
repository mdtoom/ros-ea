#!/usr/bin/env python
import time
from random import randint
from threading import Condition

import rospy
from examples.experiment_template import SingleExperiment

from run_and_visualize import ScenarioVisualiser
from simulation_control import SimulationCommunicator
from tools.score_saver import ScoreSaver


class GenomeEvaluator:
    """ This class takes care of evaluating the genomes. Note it needs to be in this file with the experiment
    (most likely because the condition)
    """

    def __init__(self, genome_encoder):

        # Find the running simulation nodes
        self.condition_lock = Condition()
        self.genome_encoder = genome_encoder
        search_string = 'score_topic'
        score_topics = [topic[0] for topic in rospy.get_published_topics() if search_string in topic[0]]
        name_spaces = [topic.replace(search_string, '') for topic in score_topics]
        self.sim_controllers = [SimulationCommunicator(ns, genome_encoder.get_message_type(), self.condition_lock)
                           for ns in name_spaces]
        time.sleep(1)       # Sleep is required for initialisation

    def get_namespaces(self):
        return [sm.namespace for sm in self.sim_controllers ]

    def finished(self):

        for sm in self.sim_controllers:
            sm.genome_publisher.unregister()

class ROSRobotExperiment(SingleExperiment):
    """ This class evaluates the genomes by sending them to a ROS node, which evaluates them."""

    def __init__(self, learning_config, num_generations, controller_keeper,
                 exp_name='', num_trails=1, base_directory='', cntrl_draw_func=None):
        """ Note the additional parameter genome_encoder, which encodes a genome for sending. """

        SingleExperiment.__init__(self, learning_config, num_generations, exp_name, num_trails,
                                  base_directory, cntrl_draw_func)
        self.controllers = controller_keeper
        self.genome_encoder = self.controllers.genome_encoder

        # Initialise the variables needed to communicate with the others trough ROS.
        self.gen_hash = 0
        rospy.init_node('evolutionary_algorithm', anonymous=True)


    def send_genomes(self, genomes):
        """ This function resends the genomes, which have not yet replied. """

        controller_count = 0
        for genome_id, genome in genomes:

            assert genome_id == genome.key  # Safety check, so both can be used as keys.
            if genome_id not in self.aggregate_scores():

                #print('Sending genome {0} to simulation {1}'.format(genome_id, controller_count))
                ros_encoded_genome = self.genome_encoder.encode(genome, self.gen_hash)
                self.controllers.sim_controllers[controller_count].publish_genome(ros_encoded_genome)
                controller_count = (controller_count + 1) % len(self.controllers.sim_controllers)

    def eval_genomes(self, genomes, config):
        start_time = time.time()

        self.gen_hash = randint(0, 1000000)  # Used to ensure that messages from a previous run are ignored.
        self.reset_robots()

        # Wait until all scores came back.
        self.controllers.condition_lock.acquire()

        previous_received = 0
        while self.not_evaluated_all(genomes):

            current_received = self.count_received()
            if previous_received == current_received:
                print('Sending missing genomes, got {0} so far'.format(previous_received))
                self.send_genomes(genomes)

            previous_received = current_received

            if rospy.is_shutdown():
                raise rospy.ROSInterruptException()

            self.controllers.condition_lock.wait(10.0)

        self.controllers.condition_lock.release()

        self.set_scores(genomes)

        end_time = time.time()
        time_diff = end_time - start_time
        avg_time = time_diff / len(genomes)

        print("generation total_runtime: %s seconds, avg_runtime: %s seconds" % (time_diff, avg_time))

    def set_scores(self, genomes):

        # Set the scores to the genomes.
        final_set = self.aggregate_scores()
        for genome_id, genome in genomes:
            genome.fitness = final_set[genome_id]

    def not_evaluated_all(self, genomes):
        return len(self.aggregate_scores()) != len(genomes)

    def reset_robots(self):

        for sc in self.controllers.sim_controllers:
            sc.reset()

    def count_received(self):
        return sum(len(sc.retrieved_scores) for sc in self.controllers.sim_controllers)

    def aggregate_scores(self):
        """ This function aggregates the scores of the simulation runners"""
        retrieved_scores = {}

        for sc in self.controllers.sim_controllers:
            retrieved_scores.update(sc.retrieved_scores)

        return retrieved_scores

    def run_full_experiment(self, num_runs):

        name = self.exp_name

        for i in range(num_runs):
            self.run(name + str(i))

        self.exp_name = name

        sv = ScenarioVisualiser(self.controllers.sim_controllers, self.base_directory, self.genome_encoder)
        sv.visualize_winner_paths()


class ROSSimultaneRobotExperiment(ROSRobotExperiment):

    def __init__(self, learning_config, num_generations, genome_encoder,
                 exp_name='', num_trails=1, base_directory='', cntrl_draw_func=None):

        ROSRobotExperiment.__init__(self, learning_config, num_generations, genome_encoder, exp_name,
                                    num_trails, base_directory, cntrl_draw_func)

        self.score_saver = ScoreSaver(self.base_directory + self.exp_name + '_scores.csv',
                                      self.controllers.get_namespaces())

    def not_evaluated_all(self, genomes):

        for sc in self.controllers.sim_controllers:
            if len(sc.retrieved_scores) != len(genomes):
                return True

        return False

    def send_genomes(self, genomes):
        """ This function resends the genomes for all genomes not yet received. """
        for sc in self.controllers.sim_controllers:
            for genome_id, genome in genomes:

                assert genome_id == genome.key  # Safety check, so both can be used as keys.
                if genome_id not in sc.retrieved_scores:

                    # print('Sending genome {0} to simulation {1}'.format(genome_id, controller_count))
                    ros_encoded_genome = self.genome_encoder.encode(genome, self.gen_hash)
                    sc.publish_genome(ros_encoded_genome)

    def set_scores(self, genomes):
        # Set the scores to the genomes.
        score_dict = {}     # Used for storing the scores.
        for genome_id, genome in genomes:

            scores = [sc.retrieved_scores[genome_id] for sc in self.controllers.sim_controllers]
            assert len(scores) == len(self.controllers.sim_controllers)
            final_score = sum(scores) / len(scores)
            score_dict[genome_id] = scores + [final_score]

            genome.fitness = final_score

        # Store scores
        self.score_saver.write_scores(self.p.generation, score_dict)

    def aggregate_scores(self):
        """ This function aggregates the scores of the simulation runners"""
        retrieved_scores = []

        for sc in self.controllers.sim_controllers:
            retrieved_scores.extend(sc.retrieved_scores[key] for key in sc.retrieved_scores)

        return retrieved_scores
