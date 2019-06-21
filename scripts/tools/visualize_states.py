# This files visualizes the states a controller has been into in a run, in order to show state usage.
# Visualization can be the states over time or the states in the environment.
import csv
import sys
import matplotlib.pyplot as plt
from matplotlib.ticker import MaxNLocator

from neat.six_util import iteritems


def visualize_scenario_state_usage(scenario_id, state_sequences):
    """ This function visualizes that state usage per controller given a scenario and a set of controllers."""

    scenario_nr = int(filter(str.isdigit, scenario_id))

    num_controllers = len(state_sequences)
    fig, axs = plt.subplots(num_controllers, 1)

    fig.suptitle('State usages of controllers in scenario {0}'.format(scenario_nr))

    for i in range(num_controllers):

        axs[i].set_ylim([0, state_sequences[i][1] - 1]) # Limit to the number of states
        axs[i].yaxis.set_major_locator(MaxNLocator(integer=True))
        axs[i].plot(state_sequences[i][2], label='controller id: {0}'.format(state_sequences[i][0]))
        axs[i].legend()

        if i != num_controllers - 1:
            plt.setp(axs[i].get_xticklabels(), visible=False)

    plt.savefig(base_dir + 'state_usage_scenario_{0}.png'.format(scenario_nr))


if __name__ == '__main__':

    if len(sys.argv) != 2:
        print('Expected as argument the folder where the file can be found.')
        exit(0)

    base_dir = sys.argv[1]

    read_state_sequences = dict()

    with open(base_dir + 'state_usages.csv', 'r') as csv_file:
        csv_reader = csv.reader(csv_file)

        for row in csv_reader:

            if row[0] not in read_state_sequences:
                read_state_sequences[row[0]] = []

            read_state_sequences[row[0]].append((int(row[1]), int(row[2]), [int(x) for x in row[3:]]))

    for sim, state_sequences in iteritems(read_state_sequences):

        visualize_scenario_state_usage(sim, state_sequences)

    plt.show()
