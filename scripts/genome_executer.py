#!/usr/bin/env python
import os
from Queue import Queue, Empty

import gym
import neat
import rospy
from examples.experiment_functions import FeedForwardNetworkController, ExperimentRunner
from ma_evolution.msg import Score, NEATGenome
from message_parsing import decode_neat_genome

genome_queue = Queue()


def callback(data):
    genome_queue.put(data)


def listener(runner, config):

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    pub = rospy.Publisher('score_topic', Score, queue_size=10)
    rospy.init_node('executioner', anonymous=True)
    rospy.Subscriber('genome_topic', NEATGenome, callback)

    rospy.loginfo('Started listener with id: %s', rospy.get_caller_id())

    while not rospy.is_shutdown():

        # Decode encoded genome.
        try:
            encoded_genome = genome_queue.get(True, 5.0)
        except Empty:
            rospy.loginfo('No genomes received for 5 seconds.')
            continue

        genome = decode_neat_genome(encoded_genome)
        fitness = runner.run(genome, config)

        rospy.loginfo(rospy.get_caller_id() + ' replies to a message from %s', encoded_genome.key)
        score_message = Score(encoded_genome.key, encoded_genome.generation, fitness)
        pub.publish(score_message)


if __name__ == '__main__':

    env_name = 'danger-zone-v0'
    num_steps = 150
    config_location = 'config-feedforward'

    local_dir = os.path.dirname(__file__)
    config_path = os.path.join(local_dir, config_location)
    config = neat.Config(neat.DefaultGenome, neat.DefaultReproduction,
                         neat.DefaultSpeciesSet, neat.DefaultStagnation,
                         config_path)

    env = gym.make(env_name)
    runner = ExperimentRunner(env, num_steps, FeedForwardNetworkController)

    listener(runner, config)


