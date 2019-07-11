import csv
import struct

import rospy
from ma_evolution.msg import SimulationReport


class StatesGatherer:
    """This class receives all the states from the evolutionary algorithm and every generation it writes them to a file.
    """

    def __init__(self, name_space, base_dir):

        stripped_ns = name_space.replace('/', "")
        self.states_file = base_dir + "sim_report_{0}.csv".format(stripped_ns)
        rospy.Subscriber(name_space + 'simreport_topic', SimulationReport, self.callback)

        with open(self.states_file, 'w'):
            pass

        self.received_state_sets = []

    def callback(self, data):
        assert len(data.state_sequence) == len(data.locations) or len(data.state_sequence) == 0

        trajectory = []
        for state, location in zip(data.state_sequence, data.locations):
            trajectory.append(location.x)
            trajectory.append(location.y)
            trajectory.append(state)

        self.received_state_sets.append([data.gen_hash, data.key, data.score, data.at_light] + trajectory)

    def write_generation(self):

        with open(self.states_file, 'a') as csv_file:
            writer = csv.writer(csv_file)

            print(self.received_state_sets[0])

            for states in self.received_state_sets:
                writer.writerow(states)
