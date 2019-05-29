#!/usr/bin/env python
import sys
from multiprocessing import Condition
from time import sleep

import rospy

from message_parsing import NEATROSEncoder, SMROSEncoder
from tools.scenario_visualizers import draw_trajectory
from simulation_control import SimulationCommunicator
from tools.score_saver import ScoreSaver
from tools.genome_analysis_tool import GenomeAnalysisTool


class ScenarioVisualiser(GenomeAnalysisTool):
    """ This class creates visualizations of each winner genome in the given class."""

    def create_visualization(self, sc, visualize_loc='scenario.png'):
        """ This method creates a visualization and writes it back to the same folder."""
        scores = []
        loc_list = []
        for genome in self.genomes:

            encoded_genome = self.encoder.encode(genome, 0)
            sc.run_genome(encoded_genome)

            # Get the result.
            trajectory = sc.get_trajectory()
            score = sc.get_score()
            scores.append(score.score)
            print('got a score of {0:.2f}'.format(score.score))
            x_locs = [loc.x for loc in trajectory.Locations]
            y_locs = [loc.y for loc in trajectory.Locations]
            loc_list.append([x_locs, y_locs, 'gen score: {0:.2f}'.format(score.score)])

        draw_trajectory(sc.namespace, loc_list, visualize_loc)

        avg_scenario_score = sum(scores) / len(scores)
        print("Average scenario score: {0}". format(avg_scenario_score))
        return avg_scenario_score

    def visualize_winner_paths(self):

        avg_scores = []
        for sc in self.sim_controllers:

            filename = 'scenario' + filter(str.isdigit, sc.namespace) + '.png'
            print(filename)
            avg_score = self.create_visualization(sc, self.base_dir + filename)
            avg_scores.append(avg_score)

        score_saver = ScoreSaver(self.base_dir + 'winner_scores.csv', [sm.namespace for sm in self.sim_controllers])

        # Write the line of average scores to a winner score csv file.
        avg_fitness = sum(avg_scores) / len(avg_scores)
        avg_scores.append(avg_fitness)
        score_dict = {'w' : avg_scores }
        score_saver.write_scores('w', score_dict)


if __name__ == '__main__':

    my_argv = rospy.myargv(argv=sys.argv)

    if len(my_argv) < 3:
        print('Expected genomes encoding an the the folder of the genome as argument.')
        exit(1)

    encoder = None
    if my_argv[1] == 'feed-forward':
        encoder = NEATROSEncoder()
        print('Going for feed-forward controller')
    elif my_argv[1] == 'state-machine':
        encoder = SMROSEncoder()
        print('Going for state-machine controller')
    else:
        print('Unknown controller class, exit.')
        exit(1)

    # Initialises the communication.
    rospy.init_node('run_genome_from_file', anonymous=True)
    condition_lock = Condition()

    # Find all the topic scores.
    search_string = 'score_topic'
    score_topics = [topic[0] for topic in rospy.get_published_topics() if search_string in topic[0]]
    name_spaces = [topic.replace(search_string, '') for topic in score_topics]
    sim_controllers = [SimulationCommunicator(ns, encoder.get_message_type(), condition_lock)
                       for ns in name_spaces]
    sleep(1)  # Sleep is required for initialisation

    sv = ScenarioVisualiser(sim_controllers, my_argv[2], encoder)
    sv.visualize_winner_paths()
