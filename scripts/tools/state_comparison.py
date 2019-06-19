# This file compares the difference between states of the files in the given folder.
import csv
import sys

import numpy as np
from neat.activations import ActivationFunctionSet
from neat.aggregations import AggregationFunctionSet
from neat.six_util import itervalues
from neat.state_machine_network import State

from genome_analysis_tool import gather_winner_genomes


def compare_states(genome):

    state_differences = dict()
    for state1 in itervalues(genome.states):

        weights1 = []
        [weights1.extend(el) for el in state1.weights]

        for state2 in itervalues(genome.states):
            key_pair = (state1.key, state2.key)

            if state1.key != state2.key and key_pair not in state_differences \
                    and key_pair[::-1] not in state_differences:
                # Only if the states are not the same
                bias_abs_diff = np.abs(state1.biases - state2.biases)

                weights2 = []
                [weights2.extend(el) for el in state2.weights]
                weights_abs_diff = np.abs(np.array(weights1) - np.array(weights2))

                diff_all = np.append(weights_abs_diff, bias_abs_diff)
                avg_diff = np.average(diff_all)
                diff_std = np.std(diff_all)

                state_differences[key_pair] = (avg_diff, diff_std)

    return state_differences


def compare_state_outcomes(genome, sensor_input_file):
    # This function evaluates the difference of the states by running it on a randomly generated set
    # input values and reporting the distance between the output vectors of the states.

    # Load the randomly generated sensor inputs
    with open(sensor_input_file, 'r') as csv_file:
        csv_reader = csv.reader(csv_file, quoting=csv.QUOTE_NONNUMERIC)

        sensor_vectors = [row for row in csv_reader]

    # Evaluate the sensor inputs on each genome.
    state_differences = dict()

    activations = ActivationFunctionSet()
    aggregations = AggregationFunctionSet()
    for enc_state1 in itervalues(genome.states):

        state1 = State(1, enc_state1.weights, enc_state1.biases, aggregations.get(enc_state1.aggregation),
                       activations.get(enc_state1.activation))

        for enc_state2 in itervalues(genome.states):

            key_pair = (enc_state1.key, enc_state2.key)
            if key_pair[0] != key_pair[1] and key_pair not in state_differences \
                    and key_pair[::-1] not in state_differences:

                state2 = State(2, enc_state2.weights, enc_state2.biases, aggregations.get(enc_state2.aggregation),
                               activations.get(enc_state2.activation))

                output_pairs = [(state1.activate(inputs), state2.activate(inputs)) for inputs in sensor_vectors]
                distances = [np.average(np.abs(np.array(output_pair[0]) - np.array(output_pair[1])))
                             for output_pair in output_pairs]

                state_differences[key_pair] = np.average(distances), np.std(distances)

    return state_differences


if __name__ == '__main__':

    if len(sys.argv) != 2:
        print("Expected a folder as argument.")
        exit(0)

    # Gather the genomes in the given directory.
    genomes = gather_winner_genomes(sys.argv[1])

    print('Comparing states based on values of the network.')
    # Evaluate the genomes
    difference_per_genome = [(genome.key, compare_states(genome)) for genome in genomes]

    for key, state_differences in difference_per_genome:
        print('{0} : {1}'.format(key, state_differences))

    print('Comparing states based on the outcomes of randomly generated sensor inputs.')

    # Evaluate the genomes
    sensor_input_location = '/home/matthijs/Desktop/random_sensors.csv'
    difference_per_genome = [(genome.key, compare_state_outcomes(genome, sensor_input_location)) for genome in genomes]

    for key, state_differences in difference_per_genome:
        print('{0} : {1}'.format(key, state_differences))
