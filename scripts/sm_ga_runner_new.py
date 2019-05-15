#!/usr/bin/env python
"""
Simple example using the tile structure creation task.
"""

import os
import neat
import rospy

from neat.reproduction_state_machine import ReproductionStateMachineOnly, StateSeparatedSpeciesSet
from neat.stagnation import MarkAllStagnation
from neat.state_machine_full_genome import StateMachineFullGenome

from message_parsing import SMROSEncoder
from ros_robot_experiment import ROSRobotExperiment, ROSSimultaneRobotExperiment

if __name__ == '__main__':

    config_location = 'config-sm_state_species'
    experiment_name = 'SM_state_dependent'
    num_generations = 100
    num_runs = 1

    # Create learning configuration.
    local_dir = os.path.dirname(__file__)
    config_path = os.path.join(local_dir, config_location)
    config = neat.Config(StateMachineFullGenome,
                         ReproductionStateMachineOnly,
                         StateSeparatedSpeciesSet,
                         MarkAllStagnation,
                         config_path)

    try:
        experiment = ROSSimultaneRobotExperiment(config, None, num_generations, SMROSEncoder, experiment_name,
                                        base_directory='/home/matthijs/Desktop/sm_new/')

        for i in range(num_runs):
            experiment.run(experiment_name + str(i))

    except rospy.ROSInterruptException:
        pass
