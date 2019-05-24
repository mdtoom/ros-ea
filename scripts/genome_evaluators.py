from multiprocessing import Condition
from random import randint
from time import sleep

import rospy

from simulation_control import SimulationCommunicator


class GenomeEvaluator:
    """ This class evaluates a genome by sending it to the desired simulator and aggregates the result."""

    def __init__(self, genome_encoder):
        rospy.init_node('evolutionary_algorithm', anonymous=True)

        self.genome_encoder = genome_encoder
        self.condition_lock = Condition()
        self.sim_controllers = None
        # Find the running simulation nodes

    def init_controllers(self):
        """ This function should initialize the controllers. Note this requires topics to be available."""
        search_string = 'score_topic'
        score_topics = [topic[0] for topic in rospy.get_published_topics() if search_string in topic[0]]
        name_spaces = [topic.replace(search_string, '') for topic in score_topics]
        self.sim_controllers = [SimulationCommunicator(ns, self.genome_encoder.get_message_type(), self.condition_lock)
                                for ns in name_spaces]

        sleep(2)       # Sleep is required for initialisation

    def send_genomes(self, genomes, gen_hash):
        """ This function resends the genomes, which have not yet replied. """

        controller_count = 0
        for genome_id, genome in genomes:

            assert genome_id == genome.key  # Safety check, so both can be used as keys.
            if genome_id not in self.aggregate_scores():

                ros_encoded_genome = self.genome_encoder.encode(genome, gen_hash)
                self.sim_controllers[controller_count].publish_genome(ros_encoded_genome)
                controller_count = (controller_count + 1) % len(self.sim_controllers)

    def simulate(self, genomes):
        """ This function simulates the given genomes. """
        previous_received = 0
        gen_hash = randint(0, 1000000)  # Used to ensure that messages from a previous run are ignored.

        self.reset()
        self.condition_lock.acquire()

        while self.not_evaluated_all(genomes):

            current_received = self.count_received()
            if previous_received == current_received:
                print('Sending missing genomes, got {0} so far'.format(previous_received))
                self.send_genomes(genomes, gen_hash)

            previous_received = current_received

            if rospy.is_shutdown():
                raise rospy.ROSInterruptException()

            self.condition_lock.wait(10.0)

        self.condition_lock.release()

        assert len(self.aggregate_scores()) == len(genomes)
        self.set_scores(genomes)

    def not_evaluated_all(self, genomes):
        """ This function returns true if not all genomes have been evaluated. """
        return self.count_received() != len(genomes)

    def reset(self):
        """ This function resets the simulation communicators."""
        for sc in self.sim_controllers:
            sc.reset()

    def count_received(self):
        """ This function sums the number of received scores for all active simulators."""
        return len(self.aggregate_scores())

    def set_scores(self, genomes):

        # Set the scores to the genomes.
        final_set = self.aggregate_scores()
        for genome_id, genome in genomes:
            genome.fitness = final_set[genome_id]

    def aggregate_scores(self):
        scores = {}

        for sc in self.sim_controllers:
            scores.update(sc.retrieved_scores)

        return scores


class AllSimulatorsGenomeEvaluator(GenomeEvaluator):
    """ This class requires the genome to be evaluated on all simulators, as each of them runs a different scenario."""

    def __init__(self, genome_encoder):

        GenomeEvaluator.__init__(self, genome_encoder)
        self.score_keeper = []

    def not_evaluated_all(self, genomes):

        for sc in self.sim_controllers:
            if len(sc.retrieved_scores) != len(genomes):
                return True

        return False

    def send_genomes(self, genomes, gen_hash):
        """ This function resends the genomes for all genomes not yet received. """
        for sc in self.sim_controllers:
            for genome_id, genome in genomes:

                assert genome_id == genome.key  # Safety check, so both can be used as keys.
                if genome_id not in sc.retrieved_scores:

                    # print('Sending genome {0} to simulation {1}'.format(genome_id, controller_count))
                    ros_encoded_genome = self.genome_encoder.encode(genome, gen_hash)
                    sc.publish_genome(ros_encoded_genome)

    def set_scores(self, genomes):
        # Set the scores to the genomes.
        score_dict = {}     # Used for storing the scores.
        for genome_id, genome in genomes:

            scores = [sc.retrieved_scores[genome_id] for sc in self.sim_controllers]
            assert len(scores) == len(self.sim_controllers)
            final_score = sum(scores) / len(scores)
            score_dict[genome_id] = scores + [final_score]

            genome.fitness = final_score

        self.score_keeper.append(score_dict)
