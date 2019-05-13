#!/usr/bin/env python
import sys
from multiprocessing import Event
from time import sleep
import matplotlib.pyplot as plt

import rospy
from gym_multi_robot.object_serializer import ObjectSerializer

from message_parsing import NEATROSEncoder, SMROSEncoder
from ros_robot_experiment import SimulationCommunicator
from ma_evolution.srv import Trajectory
from ma_evolution.srv import SimScore

event = Event()


def callback(_):

    print('score')
    event.set()


if __name__ == '__main__':

    my_argv = rospy.myargv(argv=sys.argv)

    if len(my_argv) != 3:
        print('Expected the location of the genome and its encoding as argument.')
        exit(1)

    encoder = None
    if my_argv[2] == 'feed-forward':
        encoder = NEATROSEncoder()
        print('Going for feed-forward controller')
    elif my_argv[2] == 'state-machine':
        encoder = SMROSEncoder()
        print('Going for state-machine controller')
    else:
        print('Unknown controller class, exit.')
        exit(1)

    # Load and encode the genome.
    genome = ObjectSerializer.load(my_argv[1])
    ros_encoded_genome = encoder.encode(genome, 0)

    # publish the encoder.
    rospy.init_node('run_genome_from_file', anonymous=True)

    sc = SimulationCommunicator('', encoder.get_message_type(), callback)
    sleep(1)

    sc.publish_genome(ros_encoded_genome)

    while not event.wait(1.0):

        print('yeah')
        if rospy.is_shutdown():
            exit(1)

    # Get the result.
    trajectory_service = rospy.ServiceProxy('trajectory', Trajectory)
    trajectory = trajectory_service()

    score_service = rospy.ServiceProxy('score', SimScore)
    score = score_service()
    print('got a score of {0}'.format(score))

    x_locs = [loc.x for loc in trajectory.Locations]
    y_locs = [loc.y for loc in trajectory.Locations]

    plt.plot(y_locs, x_locs)
    plt.show()






