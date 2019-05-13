#!/usr/bin/env python
import os
import rospy

import neat

from ros_robot_experiment import ROSRobotExperiment
from message_parsing import NEATROSEncoder

experiment_name = 'NEAT'
num_generations = 100
num_runs = 1

if __name__ == '__main__':

    config_location = 'config-feedforward'

    # Create learning configuration.
    local_dir = os.path.dirname(__file__)
    config_path = os.path.join(local_dir, config_location)
    config = neat.Config(neat.DefaultGenome, neat.DefaultReproduction,
                         neat.DefaultSpeciesSet, neat.DefaultStagnation,
                         config_path)


    try:
        experiment = ROSRobotExperiment(config, None, num_generations, NEATROSEncoder, experiment_name,
                                        base_directory='/home/matthijs/Desktop/obstacle_light_neat1/')

        for i in range(num_runs):
            experiment.run(experiment_name + str(i))

    except rospy.ROSInterruptException:
        pass
