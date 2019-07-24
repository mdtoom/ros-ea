#!/usr/bin/env python
import sys
from multiprocessing import Condition
from time import sleep

import rospy
import csv

from message_parsing import SMROSEncoder, NEATROSEncoder, SMSROSEncoder
from helper_functions import get_available_namespaces
from tools.visualize_states import StateUsageVisualizer
from tools.score_saver import ScoreSaver
from tools.scenario_visualizers import draw_trajectory
from simulation_control import SimulationCommunicator
from tools.genome_analysis_tool import GenomeAnalysisTool


class TrajectoryWriter:
    """ This class writes the trajectories of the given genomes in the given files."""

    def __init__(self, file_name):

        self.file_name = file_name
        with open(self.file_name, 'w'):
            pass        # Clear the csv file, since we are doing append.

    def append_trajectory(self, genome_key, score, trajectory):
        x_locs = [loc.x for loc in trajectory]
        y_locs = [loc.y for loc in trajectory]

        with open(self.file_name, 'a') as csv_file:
            csv_out = csv.writer(csv_file)
            csv_out.writerow([genome_key, score, 'x'] + x_locs)
            csv_out.writerow([genome_key, score, 'y'] + y_locs)


class StateWriter:
    """ This class writes the states to file the given genome has been in to during simulation."""

    def __init__(self, file_name):

        self.file_name = file_name
        with open(self.file_name, 'w'):
            pass        # Clear the csv file, since we are doing append.

    def append_states(self, scenario_name, genome, state_list):
        if hasattr(genome, 'states'):       # if the genome has states.
            with open(self.file_name, 'a') as csvFile:
                writer = csv.writer(csvFile)
                csv_row = [scenario_name, genome.key, len(genome.states)] + list(state_list)
                writer.writerow(csv_row)


class ScenarioVisualizer:
    """ This class visualizes the trajectory in a given file and saves it if finalize is called."""

    def __init__(self, file_name, scenario_name):
        self.file_name = file_name
        self.scenario_name = scenario_name
        self.loc_list = []

    def append_run(self, score, trajectory):
        x_locs = [loc.x for loc in trajectory]
        y_locs = [loc.y for loc in trajectory]
        self.loc_list.append([x_locs, y_locs, 'gen score: {0:.2f}'.format(score)])

    def finalize(self):
        draw_trajectory(self.scenario_name, self.loc_list, self.file_name)


class AverageScoreWriter:
    """ This class writes the average score for each scenario to a file."""

    def __init__(self, file_name, scenario_names):
        self.score_saver = ScoreSaver(file_name, scenario_names)
        self.scores_per_scenario = []

    def add_scenario_scores(self, scores):
        self.scores_per_scenario.append(scores)

    def finalize(self, entry_name='w'):

        avg_scores = [float(sum(scores)) / len(scores) for scores in self.scores_per_scenario]
        avg_fitness = sum(avg_scores) / len(avg_scores)
        avg_scores.append(avg_fitness)
        score_dict = {entry_name: avg_scores}
        self.score_saver.write_scores('w', score_dict)
        self.scores_per_scenario = []


class PostExperimentAnalysis(GenomeAnalysisTool):
    """ This class contains the analysis function that is executed when an experiment is finished."""

    def analyse(self):

        if not isinstance(self.encoder, NEATROSEncoder):
            state_writer = StateWriter(self.base_dir + 'state_usages.csv')

        avg_score_writer = AverageScoreWriter(self.base_dir + 'winner_scores.csv',
                                              [sm.namespace for sm in self.sim_controllers])

        for sc in self.sim_controllers:

            scenario_id = filter(str.isdigit, sc.namespace)

            # Init the trajectory visualizer
            visualization_file_name = 'scenario' + scenario_id + '.png'
            scenario_visualizer = ScenarioVisualizer(self.base_dir + visualization_file_name, sc.namespace)

            # Init the Trajecory writer for this scenario.
            trajectory_file_name = 'scenario' + scenario_id + '.csv'
            loc_writer = TrajectoryWriter(self.base_dir + trajectory_file_name)
            scenario_scores = []

            if not isinstance(self.encoder, NEATROSEncoder):
                state_vizualizer = StateUsageVisualizer(self.base_dir, 'scenario', scenario_id)

            for genome in self.genomes:

                encoded_genome = self.encoder.encode(genome, 0)
                sc.run_genome(encoded_genome)

                score = sc.get_score()
                trajectory = sc.get_trajectory()
                state_list = sc.get_states()

                loc_writer.append_trajectory(genome.key, score, trajectory)

                scenario_visualizer.append_run(score, trajectory)
                scenario_scores.append(score)

                if not isinstance(self.encoder, NEATROSEncoder):
                    state_vizualizer.add_genome_state_sequence(genome, state_list)
                    state_writer.append_states(sc.namespace, genome, state_list)

            scenario_visualizer.finalize()
            avg_score_writer.add_scenario_scores(scenario_scores)

            if not isinstance(self.encoder, NEATROSEncoder):
                state_vizualizer.finalize()

        avg_score_writer.finalize()


if __name__ == '__main__':

    my_argv = rospy.myargv(argv=sys.argv)

    if len(my_argv) < 2:
        print('Expected genomes encoding an the the folder of the genome as argument.')
        exit(1)

    encoder = None
    if my_argv[1] == 'feed-forward':
        encoder = NEATROSEncoder(2)
        print('Going for feed-forward controller')
    elif my_argv[1] == 'state-machine':
        encoder = SMROSEncoder()
        print('Going for state-machine controller')
    elif my_argv[1] == 'state-selector':
        encoder = SMSROSEncoder()
        print('Going for state-selector controller')
    else:
        print('Unknown controller class, exit.')
        exit(1)

    # Initialises the communication.
    rospy.init_node('run_genome_from_file', anonymous=True)
    condition_lock = Condition()

    name_spaces = get_available_namespaces()
    sim_controllers = [SimulationCommunicator(ns, encoder.get_message_type(), condition_lock)
                       for ns in name_spaces]
    sleep(1)  # Sleep is required for initialisation

    sv = PostExperimentAnalysis(sim_controllers, my_argv[2], encoder)
    sv.analyse()
