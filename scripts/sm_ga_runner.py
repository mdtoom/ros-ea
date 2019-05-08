#!/usr/bin/env python
import os

import neat
import rospy
from neat.reproduction_mutation_only import ReproductionMutationOnly
from neat.state_machine_genome import StateMachineGenome

from ros_robot_experiment import ROSRobotExperiment
from message_parsing import SMROSEncoder

if __name__ == '__main__':

    config_location = 'config-sm_free_states'
    experiment_name = 'SM_free'
    num_generations = 100
    num_runs = 5

    # Create learning configuration.
    local_dir = os.path.dirname(__file__)
    config_path = os.path.join(local_dir, config_location)
    config = neat.Config(StateMachineGenome,
                         ReproductionMutationOnly,
                         neat.DefaultSpeciesSet,
                         neat.DefaultStagnation,
                         config_path)

    try:
        experiment = ROSRobotExperiment(config, None, num_generations, SMROSEncoder, experiment_name,
                                        base_directory='/home/matthijs/Desktop/test/')

        for i in range(num_runs):
            experiment.run(experiment_name + str(i))

    except rospy.ROSInterruptException:
        pass