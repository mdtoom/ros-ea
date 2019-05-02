from ma_evolution.msg import NEATGenome, NNode, NConnection
from neat.six_util import iteritems


def encode_neat_genome(genome):
    """ This function encodes a NEAT genome in a NEATGenome ROS message. """

    encoded_nodes = []
    for node_id, node in iteritems(genome.nodes):
        encoded_node = NNode(node_id, node.bias, node.activation, node.response)
        encoded_nodes.append(encoded_node)

    encoded_connections = []
    for con_id, connection in iteritems(genome.connections):
        encoded_connection = NConnection(con_id[0], con_id[1], connection.weight, connection.enabled)
        encoded_connections.append(encoded_connection)

    encoded_genome = NEATGenome(genome.key, encoded_connections, encoded_nodes)
    return encoded_genome


def decode_neat_genome(encoded_genome):
    pass