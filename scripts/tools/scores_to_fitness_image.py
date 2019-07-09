import os
import pickle
import sys
import glob
import matplotlib.pyplot as plt

import numpy as np


def sum_and_average(folders):

    avg_max_fitnesses = []

    for folder in folders:

        stat_files = glob.glob(folder + 'stats*.pickle')
        max_per_gen = []

        print(stat_files)

        for stat_file in stat_files:
            print(stat_file)
            with open(stat_file, 'rb') as handle:
                stats = pickle.load(handle)

            best_fitnesses = [c.fitness for c in stats.most_fit_genomes]
            max_per_gen.append(best_fitnesses)

        # Make sure all runs are of equal length.
        max_generations = max(len(gen) for gen in max_per_gen)
        max_per_gen = [gen + [gen[-1]] * (max_generations - len(gen)) for gen in max_per_gen]

        print([len(x) for x in max_per_gen])
        max_per_gen = np.array(max_per_gen)

        if max_per_gen.shape[0] == 1:
            avg_max_per_gen = max_per_gen
        else:
            avg_max_per_gen = sum(max_per_gen)
            avg_max_per_gen /= max_per_gen.shape[0]

        avg_max_fitnesses.append(avg_max_per_gen)

    # Ensure that the all experiments are padded to the max number of generations.
    max_length = max(len(experiment) for experiment in avg_max_fitnesses)
    print(max_length)

    avg_max_fitnesses = [list(experiment) + [experiment[-1]] * (max_length - len(experiment)) for experiment in avg_max_fitnesses]

    max_score = max([max(gen) for gen in avg_max_fitnesses])

    # Make the plot
    for folder, avg_max_per_gen in zip(folders, avg_max_fitnesses):
        plt.plot(list(range(len(avg_max_per_gen))), avg_max_per_gen, label=folder.replace('/', ''))

    plt.xlim((0, len(avg_max_fitnesses[0])))
    plt.ylim((0, max_score + 100))
    plt.xlabel('generation')
    plt.ylabel('fitness')
    plt.title('Average fitness per generation for phototaxis with obstacles')
    plt.legend()
    plt.show()


if __name__ == '__main__':

    if len(sys.argv) < 2:
        print('expected at least one folder as argument')

    sum_and_average(sys.argv[1:])

