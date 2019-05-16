#!/usr/bin/env python
import sys
from multiprocessing import Condition
from time import sleep
import matplotlib.pyplot as plt

import rospy
from neat.object_serializer import ObjectSerializer

from message_parsing import NEATROSEncoder, SMROSEncoder
from ros_robot_experiment import SimulationCommunicator
from ma_evolution.srv import Trajectory
from ma_evolution.srv import SimScore

matplotlib_color_values = ['b', 'g', 'r', 'c', 'm', 'y', 'k']

if __name__ == '__main__':

    my_argv = rospy.myargv(argv=sys.argv)

    if len(my_argv) < 3:
        print('Expected genomes encoding an the the location(s) of the genome(s) as argument.')
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
    sc = SimulationCommunicator('', encoder.get_message_type(), condition_lock)
    sleep(1)

    num = 0
    for genome_location in my_argv[2:]:

        sc.reset()

        # Load and encode the genome.
        genome = ObjectSerializer.load(genome_location)
        ros_encoded_genome = encoder.encode(genome, 0)

        condition_lock.acquire()
        sc.publish_genome(ros_encoded_genome)

        while not condition_lock.wait(1.0):
            if len(sc.retrieved_scores) == 1:
                break

            if rospy.is_shutdown():
                exit(1)
        condition_lock.release()

        # Get the result.
        trajectory_service = rospy.ServiceProxy('trajectory', Trajectory)
        trajectory = trajectory_service()

        score_service = rospy.ServiceProxy('score', SimScore)
        score = score_service()
        print('got a score of {0}'.format(score))

        x_locs = [loc.x for loc in trajectory.Locations]
        y_locs = [loc.y for loc in trajectory.Locations]

        plt.plot(y_locs, x_locs, matplotlib_color_values[num])
        num += 1

    plt.xlim((0, 5))
    plt.ylim((-1.5, 6.5))

    plt.show()






