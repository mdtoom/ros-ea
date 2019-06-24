import rospy

from message_parsing import NEATROSEncoder, SMSROSEncoder, SMROSEncoder
from ros_robot_experiment import GenomeEvaluator, ROSSimultaneRobotExperiment
from simulation_control import SimulationController
from tools.draw_functions import draw_nn, draw_sm


def nn_based_experiment(launch_file, config, base_directory, num_generations, num_runs,
                        experiment_class=ROSSimultaneRobotExperiment):
    sim_control = SimulationController('ma_evolution', launch_file, 'feed-forward')
    sim_control.start_simulators()
    controller_keeper = None
    try:
        controller_keeper = GenomeEvaluator(NEATROSEncoder(config.genome_config.num_outputs))
        experiment = experiment_class(config, num_generations, controller_keeper,
                                      base_directory=base_directory, cntrl_draw_func=draw_nn)
        experiment.run_full_experiment(num_runs)

    except rospy.ROSInterruptException:
        pass
    finally:
        sim_control.stop_simulators()
        controller_keeper.finished()


def ss_based_experiment(launch_file, config, base_directory, num_generations, num_runs,
                        experiment_class=ROSSimultaneRobotExperiment):
    sim_control = SimulationController('ma_evolution', launch_file, 'state-selector')
    sim_control.start_simulators()
    controller_keeper = None
    try:
        controller_keeper = GenomeEvaluator(SMSROSEncoder())
        experiment = experiment_class(config, num_generations, controller_keeper, base_directory=base_directory)
        experiment.run_full_experiment(num_runs)

    except rospy.ROSInterruptException:
        pass
    finally:
        sim_control.stop_simulators()
        controller_keeper.finished()


def sm_based_experiment(launch_file, config, base_directory, num_generations, num_runs,
                        experiment_class=ROSSimultaneRobotExperiment, controller_nm='state-machine'):
    sim_control = SimulationController('ma_evolution', launch_file, controller_nm)
    sim_control.start_simulators()
    controller_keeper = None
    try:
        controller_keeper = GenomeEvaluator(SMROSEncoder())
        experiment = experiment_class(config, num_generations, controller_keeper,
                                      base_directory=base_directory, cntrl_draw_func=draw_sm)
        experiment.run_full_experiment(num_runs)

    except rospy.ROSInterruptException:
        pass
    finally:
        sim_control.stop_simulators()
        controller_keeper.finished()
