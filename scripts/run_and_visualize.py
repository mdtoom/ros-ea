#!/usr/bin/env python
import glob
import pickle
import sys
from multiprocessing import Condition
from time import sleep
import matplotlib.pyplot as plt

import rospy

from message_parsing import NEATROSEncoder, SMROSEncoder
from ros_robot_experiment import SimulationCommunicator

matplotlib_color_values = ['b', 'g', 'r', 'c', 'm', 'y', 'k']


def create_visualization(genome_locations, encoder, sc, visualize_loc='scenario.png'):
    """ This method creates a visualization and writes it back to the same folder."""
    plt.clf()

    num = 0
    for genome_location in genome_locations:

        sc.reset()

        # Load and encode the genome.
        with open(genome_location, 'rb') as handle:
            genome = pickle.load(handle)

        ros_encoded_genome = encoder.encode(genome, 0)

        sc.condition_lock.acquire()
        sc.publish_genome(ros_encoded_genome)

        while not sc.condition_lock.wait(1.0):
            if len(sc.retrieved_scores) == 1:
                break

            if rospy.is_shutdown():
                exit(1)
        sc.condition_lock.release()

        # Get the result.
        trajectory = sc.get_trajectory()
        score = sc.get_score()
        print('got a score of {0:.2f}'.format(score.score))

        x_locs = [loc.x for loc in trajectory.Locations]
        y_locs = [loc.y for loc in trajectory.Locations]

        plt.plot(y_locs, x_locs, matplotlib_color_values[num % len(matplotlib_color_values)],
                 label='gen score: {0:.2f}'.format(score.score))
        num += 1

    plt.axis('scaled')
    plt.legend()
    plt.xlim((0, 5))
    plt.ylim((-1.5, 6.5))
    plt.savefig(visualize_loc)


if __name__ == '__main__':

    my_argv = rospy.myargv(argv=sys.argv)

    if len(my_argv) < 3:
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

    genome_locations = glob.glob(my_argv[2] + 'winner*.pickle')

    for sc in sim_controllers:

        filename = 'scenario' + filter(str.isdigit, sc.namespace) + '.png'
        print(filename)
        create_visualization(genome_locations, encoder, sc, filename)
