#!/usr/bin/env python
"""
Simple example using the tile structure creation task.
"""

import os
from os.path import expanduser

import neat
from neat.generation import DefaultGeneration

from neat.reproduction_state_machine import ReproductionStateMachineOnly, StateSeparatedSpeciesSet
from neat.stagnation import MarkAllStagnation
from neat.state_selector_genome import StateSelectorGenome

from predefined_experiments import ss_based_experiment


def run_sm_selector():
    config_location = 'configs/config-sm_selector'
    num_generations = 100
    num_runs = 5
    base_directory = expanduser("~") + '/Desktop/sm_selector/'
    launch_file = 'sim_phototaxis_obst_new_fitness.launch'

    # Create learning configuration.
    local_dir = os.path.dirname(__file__)
    config_path = os.path.join(local_dir, config_location)
    config = neat.Config(StateSelectorGenome,
                         ReproductionStateMachineOnly,
                         StateSeparatedSpeciesSet,
                         MarkAllStagnation,
                         DefaultGeneration,
                         config_path)

    ss_based_experiment(launch_file, config, base_directory, num_generations, num_runs)


if __name__ == '__main__':

    run_sm_selector()
