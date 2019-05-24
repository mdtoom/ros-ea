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


class ExaggerateScenarioFourExperiment(ROSSimultaneRobotExperiment):

    def __init__(self, learning_config, num_generations, genome_encoder,
                 exp_name='', num_trails=1, base_directory='', cntrl_draw_func=None):
        ROSSimultaneRobotExperiment.__init__(self, learning_config, num_generations, genome_encoder, exp_name,
                                             num_trails, base_directory, cntrl_draw_func)

        self.exaggeration_factor = 2

    def set_scores(self, genomes):
        # Set the scores to the genomes.

        namespaces = self.controllers.get_namespaces()
        scen4 = [ns for ns in namespaces if '4' in ns]
        scen4Index = namespaces.index(scen4[0])

        score_dict = {}  # Used for storing the scores.
        for genome_id, genome in genomes:
            scores = [sc.retrieved_scores[genome_id] for sc in self.controllers.sim_controllers]
            assert len(scores) == len(self.controllers.sim_controllers)
            final_score = (sum(scores) + self.exaggeration_factor * scores[scen4Index]) \
                          / (len(scores) + self.exaggeration_factor)
            score_dict[genome_id] = scores + [final_score]
            genome.fitness = final_score

        # Store scores
        self.score_saver.write_scores(self.p.generation, score_dict)


class SuperExagerate(ExaggerateScenarioFourExperiment):

    def __init__(self, learning_config, num_generations, genome_encoder,
                 exp_name='', num_trails=1, base_directory='', cntrl_draw_func=None):
        ROSSimultaneRobotExperiment.__init__(self, learning_config, num_generations, genome_encoder, exp_name,
                                             num_trails, base_directory, cntrl_draw_func)

        self.exaggeration_factor = 10


if __name__ == '__main__':
    num_generations = 1
    num_runs = 1
    launch_file = 'sim_phototaxis_obst_multiple_sm.launch'

    simulation_base_directory = expanduser("~") + '/Desktop/four_exaggerate/'

    config_location = 'config-feedforward-no-structural'
    base_directory = simulation_base_directory + 'obstacle_light_one_layer/'

    # Create learning configuration.
    local_dir = os.path.dirname(__file__)
    config_path = os.path.join(local_dir, config_location)
    config_nn = neat.Config(neat.DefaultGenome, neat.DefaultReproduction,
                         neat.DefaultSpeciesSet, neat.DefaultStagnation,
                         config_path)

    nn_based_experiment(launch_file, config_nn, base_directory, num_generations, num_runs,
                        ExaggerateScenarioFourExperiment)

    sleep(2)

    config_location = 'config-sm_state_species'
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
                        ExaggerateScenarioFourExperiment)

    sleep(2)
    launch_file = 'sim_phototaxis_obst_new_fitness.launch'

    simulation_base_directory = expanduser("~") + '/Desktop/four_exaggerate_super/'
    base_directory = simulation_base_directory + 'obstacle_light_one_layer/'
    nn_based_experiment(launch_file, config_nn, base_directory, num_generations, num_runs,
                        SuperExagerate)

    sleep(2)

    base_directory = simulation_base_directory + 'sm_new/'
    sm_based_experiment(launch_file, config_sm_new, base_directory, num_generations, num_runs,
                        ExaggerateScenarioFourExperiment)
