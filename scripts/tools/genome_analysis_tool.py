import glob
import pickle


def gather_winner_genomes(base_dir):
    genome_locations = glob.glob(base_dir + 'winner*.pickle')
    genomes = []
    for genome_location in genome_locations:
        # Load and encode the genome.
        with open(genome_location, 'rb') as handle:
            genome = pickle.load(handle)
            genomes.append(genome)

    return genomes


class GenomeAnalysisTool:
    """ This class is the base class for analysis tools that work on winner genome files. """

    def __init__(self, sim_controllers, base_dir, encoder):
        self.sim_controllers = sim_controllers
        self.base_dir = base_dir
        self.encoder = encoder

        self.genomes = gather_winner_genomes(base_dir)