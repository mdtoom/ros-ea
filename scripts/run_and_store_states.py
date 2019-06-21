#!/usr/bin/env python
import sys
from multiprocessing import Condition
from time import sleep

import rospy
import csv

from message_parsing import SMROSEncoder, NEATROSEncoder
from simulation_control import SimulationCommunicator
from tools.genome_analysis_tool import GenomeAnalysisTool


class StatesToCSV(GenomeAnalysisTool):

    def gather_states(self):

        for sc in self.sim_controllers:

            trajectory_file_name = 'scenario' + filter(str.isdigit, sc.namespace) + '.csv'
            with open(self.base_dir + trajectory_file_name, 'w'):
                pass        # Clear the csv file, since we are doing append.

            # Clear the states file.
            if hasattr(genome, 'states'):
                with open(self.base_dir + 'state_usages.csv', 'w'):
                    pass

            for genome in self.genomes:

                self.run_genome(sc, genome)

                score = sc.get_score()
                trajectory = sc.get_trajectory()
                state_list = sc.get_states()

                x_locs = [loc.x for loc in trajectory.Locations]
                y_locs = [loc.y for loc in trajectory.Locations]

                with open(self.base_dir + trajectory_file_name, 'a') as csv_file:
                    csv_out = csv.writer(csv_file)
                    csv_out.writerow([genome.key, score.score, 'x'] + x_locs)
                    csv_out.writerow([genome.key, score.score, 'y'] + y_locs)

                if hasattr(genome, 'states'):       # if the genome has states.
                    with open(self.base_dir + 'state_usages.csv', 'a') as csvFile:
                        writer = csv.writer(csvFile)
                        csv_row = [sc.namespace, genome.key, len(genome.states)] + list(state_list.StateSequence)
                        writer.writerow(csv_row)


if __name__ == '__main__':

    my_argv = rospy.myargv(argv=sys.argv)

    if len(my_argv) < 2:
        print('Expected genomes encoding an the the folder of the genome as argument.')
        exit(1)

    encoder = None
    if my_argv[1] == 'feed-forward':
        encoder = NEATROSEncoder()
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

    sv = StatesToCSV(sim_controllers, my_argv[2], encoder)
    sv.gather_states()
