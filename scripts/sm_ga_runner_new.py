#!/usr/bin/env python
"""
Simple example using the tile structure creation task.
"""

import os
from os.path import expanduser

import neat
import rospy

from neat.reproduction_state_machine import ReproductionStateMachineOnly, StateSeparatedSpeciesSet
from neat.stagnation import MarkAllStagnation
from neat.state_machine_full_genome import StateMachineFullGenome

from message_parsing import SMROSEncoder
from ros_robot_experiment import ROSSimultaneRobotExperiment
from run_and_visualize import visualize_winner_paths


def run_sm_new():
    config_location = 'config-sm_state_species'
    experiment_name = 'SM_state_dependent'
    num_generations = 100
    num_runs = 5
    base_directory = expanduser("~") + '/Desktop/sm_new/'

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
                                                 base_directory=base_directory)

        for i in range(num_runs):
            experiment.run(experiment_name + str(i))

        visualize_winner_paths(experiment.sim_controllers, base_directory, SMROSEncoder)

    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':

    run_sm_new()