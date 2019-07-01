#!/usr/bin/env python
import os
from os.path import expanduser

import neat
from neat.generation import SeededGeneration
from neat.reproduction_mutation_only import ReproductionMutationOnly
from neat.reproduction_state_machine import StateSeparatedSpeciesSet
from neat.stagnation import MarkAllStagnation
from neat.state_machine_genome import StateMachineGenome
from predefined_experiments import sm_based_experiment


def sm_run():
    config_location = 'configs/config-sm_manual_seed'
    num_generations = 100
    num_runs = 5
    base_directory = expanduser("~") + '/Desktop/sm_seeded/'
    launch_file = 'sim_phototaxis_all.launch'

    # Create learning configuration.
    local_dir = os.path.dirname(__file__)
    config_path = os.path.join(local_dir, config_location)
    config = neat.Config(StateMachineGenome,
                         ReproductionMutationOnly,
                         StateSeparatedSpeciesSet,
                         MarkAllStagnation,
                         SeededGeneration,
                         config_path)

    sm_based_experiment(launch_file, config, base_directory, num_generations, num_runs)


if __name__ == '__main__':

    sm_run()
