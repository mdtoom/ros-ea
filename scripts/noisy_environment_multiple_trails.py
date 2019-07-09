#!/usr/bin/env python
import sys
from multiprocessing import Condition
from time import sleep

import rospy

from message_parsing import NEATROSEncoder, SMROSEncoder
from post_experiment_analysis import AverageScoreWriter
from tools.genome_analysis_tool import GenomeAnalysisTool
from simulation_control import SimulationCommunicator


class GetMultipleTrailScore(GenomeAnalysisTool):
    """ This class runs multiple trails of the same controllers and outputs the average scores."""

    def analyse(self, num_runs):

        avg_score_writer = AverageScoreWriter(self.base_dir + 'multiple_winner_scores.csv',
                                              [sm.namespace for sm in self.sim_controllers])

        passes = {x.namespace: 0 for x in self.sim_controllers}

        # For every genome calculate the average score for n runs on all scenarios.
        for genome in self.genomes:

            encoded_genome = self.encoder.encode(genome, 0)

            for sc in self.sim_controllers:
                scenario_scores = []

                for i in range(num_runs):

                    sc.run_genome(encoded_genome)
                    score = sc.get_score()
                    scenario_scores.append(score)

                    passes[sc.namespace] += sc.get_at_light()

                print(sc.namespace)
                print(scenario_scores)
                avg_score_writer.add_scores(scenario_scores)

            avg_score_writer.finalize(genome.key)

        print(passes)


if __name__ == '__main__':

    my_argv = rospy.myargv(argv=sys.argv)
    num_runs = 100

    if len(my_argv) < 2:
        print('Expected genomes encoding an the the folder of the genome as argument.')
        exit(1)

    encoder = None
    if my_argv[1] == 'feed-forward':
        encoder = NEATROSEncoder(2)
        print('Going for feed-forward controller')
    elif my_argv[1] == 'state-machine':
        encoder = SMROSEncoder()
        print('Going for state-machine controller')
    else:
        print('Unknown controller class, exit.')
        exit(1)

    # Initialises the communication.
    rospy.init_node('run_genome_from_file', anonymous=True)
    condition_lock = Condition()

    # Find all the topic scores.
    search_string = 'score_topic'
    score_topics = [topic[0] for topic in rospy.get_published_topics() if search_string in topic[0]]
    name_spaces = [topic.replace(search_string, '') for topic in score_topics]
    sim_controllers = [SimulationCommunicator(ns, encoder.get_message_type(), condition_lock)
                       for ns in name_spaces]
    sleep(1)  # Sleep is required for initialisation

    sv = GetMultipleTrailScore(sim_controllers, my_argv[2], encoder)
    sv.analyse(num_runs)