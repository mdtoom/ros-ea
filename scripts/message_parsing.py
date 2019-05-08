import numpy as np

from ma_evolution.msg import NEATGenome, NNode, NConnection, SMGenome, SMState, SMTransition, SMCondition, WeightVector
from neat import DefaultGenome
from neat.genes import DefaultConnectionGene, DefaultNodeGene
from neat.six_util import iteritems
from neat.state_machine_genes import TransitionGene, StateGene
from neat.state_machine_genome import StateMachineGenome
from neat.state_machine_network import Condition

class ROSEncoder:
    """ This class provides the interface for a ROS encoder and can be used to encode/decode a genome"""

    @staticmethod
    def encode(genome, generation):
        """ This function should encode a genome. """
        pass

    @staticmethod
    def decode(encoded_genome):
        """ This function should decode a genome. """
        pass

    @staticmethod
    def get_message_type():
        """ This function should return the ROS message type of an encoded genome. """


class NEATROSEncoder(ROSEncoder):
    """ This class provides the function required to encode and decode NEAT genomes."""

    @staticmethod
    def encode(genome, generation):
        """ This function encodes a NEAT genome in a NEATGenome ROS message. """

        encoded_nodes = []
        for node_id, node in iteritems(genome.nodes):
            encoded_node = NNode(node_id, node.bias, node.activation, node.aggregation, node.response)
            encoded_nodes.append(encoded_node)

        encoded_connections = []
        for con_id, connection in iteritems(genome.connections):
            encoded_connection = NConnection(con_id[0], con_id[1], connection.weight, connection.enabled)
            encoded_connections.append(encoded_connection)

        encoded_genome = NEATGenome(genome.key, generation, encoded_connections, encoded_nodes)
        return encoded_genome

    @staticmethod
    def decode(encoded_genome):
        """ This function decodes a message received trough a topic using """

        nodes = {}
        for enc_node in encoded_genome.nodes:
            nodes[enc_node.key] = DefaultNodeGene(enc_node.key)
            nodes[enc_node.key].bias = enc_node.bias
            nodes[enc_node.key].activation = enc_node.activation
            nodes[enc_node.key].aggregation = enc_node.aggregation
            nodes[enc_node.key].response = enc_node.response

        connections = {}
        for enc_connection in encoded_genome.connections:
            key = (enc_connection.source, enc_connection.dest)
            connections[key] = DefaultConnectionGene(key)
            connections[key].weight = enc_connection.weight
            connections[key].enabled = enc_connection.enabled

        genome = DefaultGenome(encoded_genome.key)
        genome.connections = connections
        genome.nodes = nodes

        return genome

    @staticmethod
    def get_message_type():
        return NEATGenome


class SMROSEncoder(ROSEncoder):
    """ This class provides the functions required to encode and decode SM messages. """

    @staticmethod
    def encode(genome, generation):
        """ This function encodes a State Machine genome to be send over ROS. """

        encoded_states = []
        for state_id, state in iteritems(genome.states):
            weight_vector = [WeightVector(weight_array) for weight_array in state.weights]

            encoded_state = SMState(state_id, state.biases, weight_vector, state.activation, state.aggregation)
            encoded_states.append(encoded_state)

        encoded_transitions = []
        for transition_id, transition in iteritems(genome.transitions):
            enc_conditions = [SMCondition(condition[0], Condition.op_to_int(condition[1]), condition[2])
                              for condition in transition.conditions]
            encoded_transition = SMTransition(transition_id[0], transition_id[1], transition.enabled, enc_conditions)
            encoded_transitions.append(encoded_transition)

        encoded_genome = SMGenome(genome.key, generation, encoded_transitions, encoded_states)
        return encoded_genome

    @staticmethod
    def decode(encoded_genome):
        """ This function decodes a ROS encoded state machine genome into a normal state machine genome."""

        states = {}
        for enc_state in encoded_genome.states:
            states[enc_state.key] = StateGene(enc_state.key)
            states[enc_state.key].biases = np.array(enc_state.biases)
            states[enc_state.key].weights = np.array([weight_vector.weights
                                                      for weight_vector in enc_state.weight_vec])
            states[enc_state.key].aggregation = enc_state.aggregation
            states[enc_state.key].activation = enc_state.activation

        transitions = {}
        for enc_transition in encoded_genome.transitions:
            key = (enc_transition.source, enc_transition.dest)
            transitions[key] = TransitionGene(key)
            transitions[key].enabled = enc_transition.enabled
            transitions[key].conditions = [(enc_condition.inputSensor, Condition.int_to_op(enc_condition.operator),
                                            enc_condition.comparator_value)
                                           for enc_condition in enc_transition.conditions]

        genome = StateMachineGenome(encoded_genome.key)
        genome.transitions = transitions
        genome.states = states

        return genome

    @staticmethod
    def get_message_type():
        return SMGenome
