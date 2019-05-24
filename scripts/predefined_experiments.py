import rospy

from message_parsing import NEATROSEncoder, SMSROSEncoder, SMROSEncoder
from ros_robot_experiment import GenomeEvaluator, ROSSimultaneRobotExperiment
from run_and_store_states import StatesToCSV
from simulation_control import SimulationController
from tools.draw_functions import draw_nn


def nn_based_experiment(launch_file, config, base_directory, num_generations, num_runs,
                        experiment_class=ROSSimultaneRobotExperiment):
    sim_control = SimulationController('ma_evolution', launch_file, 'feed-forward')
    sim_control.start_simulators()
    try:
        controller_keeper = GenomeEvaluator(NEATROSEncoder)
        experiment = experiment_class(config, num_generations, controller_keeper,
                                      base_directory=base_directory, cntrl_draw_func=draw_nn)
        experiment.run_full_experiment(num_runs)

    except rospy.ROSInterruptException:
        pass
    finally:
        sim_control.stop_simulators()


def ss_based_experiment(launch_file, config, base_directory, num_generations, num_runs,
                        experiment_class=ROSSimultaneRobotExperiment):
    sim_control = SimulationController('ma_evolution', launch_file, 'state-selector')
    sim_control.start_simulators()
    try:
        controller_keeper = GenomeEvaluator(SMSROSEncoder)
        experiment = experiment_class(config, num_generations, controller_keeper, base_directory=base_directory)
        experiment.run_full_experiment(num_runs)

        stc = StatesToCSV(controller_keeper.sim_controllers, base_directory, SMSROSEncoder)
        stc.gather_states()

    except rospy.ROSInterruptException:
        pass
    finally:
        sim_control.stop_simulators()


def sm_based_experiment(launch_file, config, base_directory, num_generations, num_runs,
                        experiment_class=ROSSimultaneRobotExperiment):
    sim_control = SimulationController('ma_evolution', launch_file, 'state-machine')
    sim_control.start_simulators()
    try:
        controller_keeper = GenomeEvaluator(SMROSEncoder)
        experiment = experiment_class(config, num_generations, controller_keeper,
                                      base_directory=base_directory)
        experiment.run_full_experiment(num_runs)

        stc = StatesToCSV(controller_keeper.sim_controllers, base_directory, SMROSEncoder)
        stc.gather_states()

    except rospy.ROSInterruptException:
        pass
    finally:
        sim_control.stop_simulators()
