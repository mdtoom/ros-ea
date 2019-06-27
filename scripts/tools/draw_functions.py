from subprocess import CalledProcessError

from neat import visualize


def draw_sm(genome, config, file_name='winner'):
    """ This function draws a state machine. """
    try:
        visualize.draw_state_machine(config, genome, filename=file_name)
    except CalledProcessError:
        print('State machine graph failed, continuing without producing graph.')


def draw_nn(genome, config, file_name='winner.svg'):
    """ This function draws a neural network. """
    visualize.draw_net(config, genome, filename=file_name)
