import pickle
import sys
from subprocess import CalledProcessError
from neat import visualize


def draw_sm(genome, config, file_name='winner.svg'):
    """ This function draws a state machine. """
    try:
        visualize.draw_state_machine(config, genome, filename=file_name)
    except CalledProcessError:
        print('State machine graph failed, continuing without producing graph.')


def draw_nn(genome, config, file_name='winner.svg'):
    """ This function draws a neural network. """
    visualize.draw_net(config, genome, filename=file_name)


if __name__ == '__main__':

    if len(sys.argv) < 3:
        print('Expected the type of controller and at least one controller file.')

    controller_name = sys.argv[1]
    draw_function = None
    if controller_name == 'state-machine':
        draw_function = draw_sm
    elif controller_name == 'neural-network':
        draw_function = draw_nn

    for controller_file in sys.argv[2:]:
        with open(controller_file, 'rb') as handle:
            controller = pickle.load(handle)


