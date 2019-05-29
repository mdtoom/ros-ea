# This file adds visualizations for the different scenarios in matplotlib. It indicates the obstacles.
from matplotlib.collections import PatchCollection
from matplotlib.patches import Rectangle

import matplotlib.pyplot as plt


def draw_trajectory(scenario_nm, loc_list, output_file):
    """ This function draws the given trajectories in the correct simulation."""
    plt.clf()
    draw_scenario(scenario_nm, plt.gca())

    num = 0
    matplotlib_color_values = ['b', 'g', 'r', 'c', 'm', 'y', 'k']

    for locations in loc_list:

        x_locs = locations[0]
        y_locs = locations[1]

        plt.plot(y_locs, x_locs, matplotlib_color_values[num % len(matplotlib_color_values)],
                 label=locations[2])
        num += 1

    plt.axis('scaled')
    plt.legend()
    plt.xlim((0, 5))
    plt.ylim((-1.5, 6.5))
    plt.savefig(output_file)


def draw_scenario(namespace, axis):
    """ This function draws a scenario given a namespace."""

    draw_function = borders

    if '4' in namespace:
        draw_function = scenario4
    elif '3' in namespace:
        draw_function = scenario3
    elif '2' in namespace:
        draw_function = scenario2

    draw_function(axis)


def borders(ax):
    """ This function draws the borders of a normal 6x9 ros grid."""
    patches = [Rectangle((-0.05, -1.55), 0.1, 8), Rectangle((4.95, -1.55), 0.1, 8),
               Rectangle((-0.05, -1.55), 5, 0.1), Rectangle((-0.05, 6.45), 5, 0.1)]

    # patches = [Rectangle((5, 2.5), 0.1, 8), Rectangle((0, 2.5), 0.1, 8),
    #            Rectangle((-1.5, 2.5), 0.1, 5), Rectangle((6.5, 2.5), 0.1, 5)]

    collection = PatchCollection(patches)
    ax.add_collection(collection)


def scenario2(ax):
    """ This function draws the normal scenario 2. """
    borders(ax)

    patches = [Rectangle((2.25, 2.75), 0.5, 0.5), Rectangle((1.25, -1.25), 2.5, 0.5),
               Rectangle((1.25, -1.0), 0.5, 2.0), Rectangle((3.25, -1.0), 0.5, 2.0)]

    collection = PatchCollection(patches)
    ax.add_collection(collection)


def scenario3(ax):
    """ This function draws the normal scenario 3. """
    borders(ax)

    patches = [Rectangle((1.75, 2.25), 1.5, 0.5),
               Rectangle((0.75, 0.5), 0.5, 1.0),
               Rectangle((3.75, 0.5), 0.5, 1.0)]

    collection = PatchCollection(patches)
    ax.add_collection(collection)


def scenario4(ax):
    borders(ax)

    patches = [Rectangle((1.25, 1.25), 2.5, 0.5),
               Rectangle((1.25, -0.5), 0.5, 2.0),
               Rectangle((3.25, -0.5), 0.5, 2.0)]

    collection = PatchCollection(patches)
    ax.add_collection(collection)


if __name__ == '__main__':

    ax = plt.gca()
    scenario4(ax)

    plt.axis('scaled')
    plt.legend()
    plt.xlim((-0.5, 5.5))
    plt.ylim((-2.0, 7.0))

    plt.show()
