#!/usr/bin/env python
import os
from os.path import expanduser
from time import sleep

import neat
from neat.reproduction_state_machine import ReproductionStateMachineOnly, StateSeparatedSpeciesSet
from neat.stagnation import MarkAllStagnation
from neat.state_machine_full_genome import StateMachineFullGenome

from predefined_experiments import nn_based_experiment, sm_based_experiment
from ros_robot_experiment import ROSSimultaneRobotExperiment

if __name__ == '__main__':

    num_generations = 100
    num_runs = 5
    launch_file = 'sim_phototaxis_obst_multiple_sm.launch'

    simulation_base_directory = expanduser("~") + '/Desktop/tanh_verification/'

    # config_location = 'config-feedforward-no-structural'
    # base_directory = simulation_base_directory + 'obstacle_light_one_layer/'
    #
    # # Create learning configuration.
    # local_dir = os.path.dirname(__file__)
    # config_path = os.path.join(local_dir, config_location)
    # config_nn = neat.Config(neat.DefaultGenome, neat.DefaultReproduction,
    #                         neat.DefaultSpeciesSet, neat.DefaultStagnation,
    #                         config_path)
    #
    # nn_based_experiment(launch_file, config_nn, base_directory, num_generations, num_runs,
    #                     ROSSimultaneRobotExperiment)

    sleep(2)

    config_location = 'configs/config-sm_one_state'
    base_directory = simulation_base_directory + 'sm_new/'

    # Create learning configuration.
    local_dir = os.path.dirname(__file__)
    config_path = os.path.join(local_dir, config_location)
    config_sm_new = neat.Config(StateMachineFullGenome,
                                ReproductionStateMachineOnly,
                                StateSeparatedSpeciesSet,
                                MarkAllStagnation,
                                config_path)

    sm_based_experiment(launch_file, config_sm_new, base_directory, num_generations, num_runs,
                        ROSSimultaneRobotExperiment)
