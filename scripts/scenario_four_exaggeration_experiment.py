#!/usr/bin/env python
import os
from os.path import expanduser
from time import sleep

import neat
from neat.reproduction_mutation_only import ReproductionMutationOnly
from neat.reproduction_state_machine import ReproductionStateMachineOnly, StateSeparatedSpeciesSet
from neat.stagnation import MarkAllStagnation
from neat.state_machine_full_genome import StateMachineFullGenome
from neat.state_machine_genome import StateMachineGenome
from neat.state_selector_genome import StateSelectorGenome

from predefined_experiments import nn_based_experiment, sm_based_experiment, ss_based_experiment
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


class SuperExaggerate(ExaggerateScenarioFourExperiment):

    def __init__(self, learning_config, num_generations, genome_encoder,
                 exp_name='', num_trails=1, base_directory='', cntrl_draw_func=None):
        ROSSimultaneRobotExperiment.__init__(self, learning_config, num_generations, genome_encoder, exp_name,
                                             num_trails, base_directory, cntrl_draw_func)

        self.exaggeration_factor = 10


class SemiSuperExaggerate(ExaggerateScenarioFourExperiment):

    def __init__(self, learning_config, num_generations, genome_encoder,
                 exp_name='', num_trails=1, base_directory='', cntrl_draw_func=None):
        ROSSimultaneRobotExperiment.__init__(self, learning_config, num_generations, genome_encoder, exp_name,
                                             num_trails, base_directory, cntrl_draw_func)

        self.exaggeration_factor = 5


if __name__ == '__main__':
    num_generations = 100
    num_runs = 5

    # Create learning configuration.
    nn_config_location = 'configs/config-feedforward-no-structural'
    local_dir = os.path.dirname(__file__)
    config_path = os.path.join(local_dir, nn_config_location)
    config_nn = neat.Config(neat.DefaultGenome, neat.DefaultReproduction,
                            neat.DefaultSpeciesSet, neat.DefaultStagnation,
                            config_path)

    # Create learning configuration.
    # sm_config_location = 'config-sm_state_species'
    # local_dir = os.path.dirname(__file__)
    # config_path = os.path.join(local_dir, sm_config_location)
    # config_sm_new = neat.Config(StateMachineFullGenome,
    #                             ReproductionStateMachineOnly,
    #                             StateSeparatedSpeciesSet,
    #                             MarkAllStagnation,
    #                             config_path)

    # Create learning configuration.
    sm_free_config_location = 'configs/config-sm_free_states'
    config_path = os.path.join(local_dir, sm_free_config_location)
    config_sm = neat.Config(StateMachineGenome,
                         ReproductionMutationOnly,
                         neat.DefaultSpeciesSet,
                         neat.DefaultStagnation,
                         config_path)

    launch_file = 'sim_phototaxis_obst_new_fitness.launch'

    # Redo of the super exaggerate problem.
    simulation_base_directory = expanduser("~") + '/Desktop/four_exaggerate_super/'
    base_directory = simulation_base_directory + 'sm/'
    sm_based_experiment(launch_file, config_sm, base_directory, num_generations, num_runs,
                        SuperExaggerate)

    sleep(10)

    base_directory = simulation_base_directory + 'state_selector/'

    ss_config_location = 'configs/config-sm_selector'
    config_path = os.path.join(local_dir, ss_config_location)
    config = neat.Config(StateSelectorGenome,
                         ReproductionStateMachineOnly,
                         StateSeparatedSpeciesSet,
                         MarkAllStagnation,
                         config_path)

    ss_based_experiment(launch_file, config, base_directory, num_generations, num_runs,
                        SuperExaggerate)

    # base_directory = simulation_base_directory + 'nn/'
    # nn_based_experiment(launch_file, config_nn, base_directory, num_generations, num_runs,
    #                     SemiSuperExaggerate)
