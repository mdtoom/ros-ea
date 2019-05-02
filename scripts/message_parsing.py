from ma_evolution.msg import NEATGenome, NNode, NConnection
from neat import DefaultGenome
from neat.genes import DefaultConnectionGene, DefaultNodeGene
from neat.six_util import iteritems


def encode_neat_genome(genome, generation):
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


def decode_neat_genome(encoded_genome):
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