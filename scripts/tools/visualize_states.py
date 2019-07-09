# This files visualizes the states a controller has been into in a run, in order to show state usage.
# Visualization can be the states over time or the states in the environment.
import csv
import sys
import matplotlib.pyplot as plt
from matplotlib.ticker import MaxNLocator

from neat.six_util import iteritems

selection_opposite = {'controller': 'scenario', 'scenario': 'controller'}


class StateUsageVisualizer:

    def __init__(self, base_dir, selection_type, group_id):
        self.base_dir = base_dir
        self.selection_type = selection_type
        self.group_id = group_id
        self.state_sequences = []

    def add_state_sequence(self, sequence_id, num_states, state_sequence):
        self.state_sequences.append((sequence_id, num_states, state_sequence))

    def add_genome_state_sequence(self, genome, state_sequence):
        if hasattr(genome, 'states'):
            self.state_sequences.append((genome.key, len(genome.states), state_sequence))

    def finalize(self):
        """ This function visualizes that state usage per controller given a scenario and a set of controllers."""
        len_of_group = len(self.state_sequences)
        plt.clf()
        fig, axs = plt.subplots(len_of_group, 1)

        # In case only one controller is tested put axs in a list
        if len_of_group == 1:
            axs = [axs]

        fig.suptitle('State usages of {0} {1}'.format(self.selection_type, self.group_id))

        for i in range(len_of_group):
            axs[i].set_ylim([0, self.state_sequences[i][1] - 0.96])   # Limit to the number of states
            axs[i].yaxis.set_major_locator(MaxNLocator(integer=True))
            axs[i].plot(self.state_sequences[i][2],
                        label='{0}: {1}'.format(selection_opposite[self.selection_type], self.state_sequences[i][0]))
            axs[i].legend()

            if i != len_of_group - 1:
                plt.setp(axs[i].get_xticklabels(), visible=False)

        plt.xlabel('time steps')
        fig.text(0.06, 0.5, 'State', ha='center', va='center', rotation='vertical')

        plt.savefig(self.base_dir + 'state_usage_{0}_{1}.png'.format(self.selection_type, self.group_id), bbox_inches='tight')


if __name__ == '__main__':

    if len(sys.argv) != 3:
        print('Expected arguments: selection type (scenario | controller) and the folder where the file can be found.')
        exit(0)

    selection_type = sys.argv[1]
    base_dir = sys.argv[2]

    read_state_sequences = dict()

    with open(base_dir + 'state_usages.csv', 'r') as csv_file:
        csv_reader = csv.reader(csv_file)

        for row in csv_reader:

            if selection_type == 'scenario':
                key = int(int(filter(str.isdigit, row[0])))
                index = int(row[1])
            elif selection_type == 'controller':
                key = int(row[1])
                index = int(int(filter(str.isdigit, row[0])))
            else:
                print('Unknown selection type')
                exit(1)

            if key not in read_state_sequences:
                read_state_sequences[key] = []

            read_state_sequences[key].append((index, int(row[2]), [int(x) for x in row[3:]]))

    for key, state_sequences in iteritems(read_state_sequences):

        visualizer = StateUsageVisualizer(base_dir, selection_type, key)
        visualizer.state_sequences = state_sequences    # Cheat to not have to append them all.
        visualizer.finalize()
