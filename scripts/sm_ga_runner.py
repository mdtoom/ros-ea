#!/usr/bin/env python
import os
from os.path import expanduser

import neat
import rospy
from neat.reproduction_mutation_only import ReproductionMutationOnly
from neat.state_machine_genome import StateMachineGenome

from ros_robot_experiment import ROSSimultaneRobotExperiment
from message_parsing import SMROSEncoder
from run_and_visualize import ScenarioVisualiser
from tools.draw_functions import draw_sm


def sm_run():
    config_location = 'config-sm_free_states'
    experiment_name = 'SM_free'
    num_generations = 100
    num_runs = 5
    base_directory = expanduser("~") + '/Desktop/sm/'

    # Create learning configuration.
    local_dir = os.path.dirname(__file__)
    config_path = os.path.join(local_dir, config_location)
    config = neat.Config(StateMachineGenome,
                         ReproductionMutationOnly,
                         neat.DefaultSpeciesSet,
                         neat.DefaultStagnation,
                         config_path)

    try:
        experiment = ROSSimultaneRobotExperiment(config, num_generations, SMROSEncoder, experiment_name,
                                        base_directory=base_directory, cntrl_draw_func=draw_sm)

        for i in range(num_runs):
            experiment.run(experiment_name + str(i))

        sv = ScenarioVisualiser(experiment.sim_controllers, base_directory, SMROSEncoder)
        sv.visualize_winner_paths()

    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':

    sm_run()
