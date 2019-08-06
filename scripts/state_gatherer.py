import csv

import rospy
from ma_evolution.msg import SimulationReport
from neat.six_util import iteritems, itervalues

from tools.helper_functions import create_file


class StatesGatherer:
    """This class receives all the states from the evolutionary algorithm and every generation it writes them to a file.
    """

    def __init__(self, name_space, base_dir):

        stripped_ns = name_space.replace('/', "")
        self.states_file = base_dir + "sim_report_{0}.csv".format(stripped_ns)
        rospy.Subscriber(name_space + 'simreport_topic', SimulationReport, self.callback)

        create_file(self.states_file)
        self.received_state_sets = {}
        self.current_generation = 0

    def set_current_generation(self, generation):
        """ Set the current generation for skipping unwanted messages."""
        self.current_generation = generation
        self.received_state_sets = {}

    def callback(self, data):
        assert len(data.state_sequence) == len(data.locations) or len(data.state_sequence) == 0

        if data.header.generation == self.current_generation \
                and (data.header.generation, data.header.key) not in self.received_state_sets:

            state_dict = {}

            for state in data.state_sequence:
                if state not in state_dict:
                    state_dict[state] = 0

                state_dict[state] += 1

            state_usage = []
            for key, value in iteritems(state_dict):
                state_usage.append(key)
                state_usage.append(value)

            self.received_state_sets[(data.header.generation, data.header.key)] = \
                [data.header.generation, data.header.key, data.score, data.at_light] + state_usage

    def write_generation(self):

        with open(self.states_file, 'a') as csv_file:
            writer = csv.writer(csv_file)

            for states in itervalues(self.received_state_sets):
                writer.writerow(states)
