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
from neat.state_selector_genome import StateSelectorGenome

from message_parsing import SMSROSEncoder
from ros_robot_experiment import ROSSimultaneRobotExperiment
from run_and_visualize import ScenarioVisualiser
from run_and_store_states import StatesToCSV


def run_sm_selector():
    config_location = 'config-sm_selector'
    experiment_name = 'SM_state_dependent'
    num_generations = 100
    num_runs = 5
    base_directory = expanduser("~") + '/Desktop/sm_selector/'

    # Create learning configuration.
    local_dir = os.path.dirname(__file__)
    config_path = os.path.join(local_dir, config_location)
    config = neat.Config(StateSelectorGenome,
                         ReproductionStateMachineOnly,
                         StateSeparatedSpeciesSet,
                         MarkAllStagnation,
                         config_path)

    try:
        experiment = ROSSimultaneRobotExperiment(config, num_generations, SMSROSEncoder, experiment_name,
                                                 base_directory=base_directory)

        for i in range(num_runs):
            experiment.run(experiment_name + str(i))

        print('going to visualisation')

        sv = ScenarioVisualiser(experiment.sim_controllers, base_directory, SMSROSEncoder)
        sv.visualize_winner_paths()

        ss = StatesToCSV(experiment.sim_controllers, base_directory, SMSROSEncoder)
        ss.gather_states()

    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':

    run_sm_selector()
