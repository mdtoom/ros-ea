#!/usr/bin/env python
import os
from os.path import expanduser

import neat

from neat.reproduction_state_machine import ReproductionStateMachineOnly, StateSeparatedSpeciesSet
from neat.stagnation import MarkAllStagnation
from neat.state_machine_full_genome import StateMachineFullGenome

from predefined_experiments import sm_based_experiment

def fixed_2_state_experiment():
    config_location = 'configs/config-sm_fixed_two_states'
    num_generations = 100
    num_runs = 5
    base_directory = expanduser("~") + '/Desktop/sm_new/'
    launch_file = 'sim_phototaxis_obst_new_fitness.launch'

    # Create learning configuration.
    local_dir = os.path.dirname(__file__)
    config_path = os.path.join(local_dir, config_location)
    config = neat.Config(StateMachineFullGenome,
                         ReproductionStateMachineOnly,
                         StateSeparatedSpeciesSet,
                         MarkAllStagnation,
                         config_path)

    sm_based_experiment(launch_file, config, base_directory, num_generations, num_runs, controller_nm='fixed-2-states')


if __name__ == '__main__':

    fixed_2_state_experiment()
