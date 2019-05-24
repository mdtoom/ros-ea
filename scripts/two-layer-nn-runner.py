#!/usr/bin/env python
import os
from os.path import expanduser

import rospy

import neat

from ros_robot_experiment import ROSRobotExperiment, ROSSimultaneRobotExperiment, GenomeEvaluator
from message_parsing import NEATROSEncoder
from simulation_control import SimulationController
from tools.draw_functions import draw_nn


def two_layer_run():

    num_generations = 1
    num_runs = 1
    config_location = 'config-feedforward-2-hidden-layers'
    base_directory = expanduser("~") + '/Desktop/two-layer-nn/'

    # Create learning configuration.
    local_dir = os.path.dirname(__file__)
    config_path = os.path.join(local_dir, config_location)
    config = neat.Config(neat.DefaultGenome, neat.DefaultReproduction,
                         neat.DefaultSpeciesSet, neat.DefaultStagnation,
                         config_path)

    sim_control = SimulationController('ma_evolution', 'sim_phototaxis_obst_new_fitness.launch', 'feed-forward')
    sim_control.start_simulators()
    try:
        controller_keeper = GenomeEvaluator(NEATROSEncoder)
        experiment = ROSRobotExperiment(config, num_generations, controller_keeper,
                                                 base_directory=base_directory, cntrl_draw_func=draw_nn)
        experiment.run_full_experiment(num_runs)

    except rospy.ROSInterruptException:
        pass
    finally:
        sim_control.stop_simulators()


if __name__ == '__main__':

    two_layer_run()
