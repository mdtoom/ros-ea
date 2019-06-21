import glob
import pickle
import rospy


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

    def run_genome(self, sc, genome):

        sc.reset()
        ros_encoded_genome = self.encoder.encode(genome, 0)

        sc.condition_lock.acquire()
        sc.publish_genome(ros_encoded_genome)

        while not sc.condition_lock.wait(1.0):
            if len(sc.retrieved_scores) == 1:
                break

            if rospy.is_shutdown():
                exit(1)
        sc.condition_lock.release()