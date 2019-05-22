#!/usr/bin/env python
import sys
from multiprocessing import Condition
from time import sleep

import rospy
import csv

from ros_robot_experiment import SimulationCommunicator
from message_parsing import SMROSEncoder
from tools.genome_analysis_tool import GenomeAnalysisTool
from ma_evolution.srv import StateRequest


class StatesToCSV(GenomeAnalysisTool):

    def gather_states(self):

        with open(self.base_dir + 'state_usages.csv', 'a') as csvFile:
            writer = csv.writer(csvFile)
            for sc in self.sim_controllers:

                for genome in self.genomes:

                    self.run_genome(sc, genome)

                    rospy.wait_for_service(sc.namespace + 'states_request')
                    state_service = rospy.ServiceProxy(sc.namespace + 'states_request', StateRequest)
                    state_list = state_service()

                    csv_row = [sc.namespace, genome.key, len(genome.states)] + list(state_list.StateSequence)
                    writer.writerow(csv_row)


if __name__ == '__main__':

    my_argv = rospy.myargv(argv=sys.argv)

    if len(my_argv) < 2:
        print('Expected the folder of the genome as argument.')
        exit(1)

    # Initialises the communication.
    rospy.init_node('run_genome_from_file', anonymous=True)
    condition_lock = Condition()

    # Find all the topic scores.
    search_string = 'score_topic'
    score_topics = [topic[0] for topic in rospy.get_published_topics() if search_string in topic[0]]
    name_spaces = [topic.replace(search_string, '') for topic in score_topics]
    sim_controllers = [SimulationCommunicator(ns, SMROSEncoder.get_message_type(), condition_lock)
                       for ns in name_spaces]
    sleep(1)  # Sleep is required for initialisation

    sv = StatesToCSV(sim_controllers, my_argv[1], SMROSEncoder)
    sv.gather_states()
