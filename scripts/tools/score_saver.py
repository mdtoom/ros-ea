import csv
from neat.six_util import iteritems


class ScoreSaver:
    """ This class receives a set of genomes and their scores and writes it to the given CSV file (append).
    The layout of the csv is:
    generation id score_1 score_2 ... score_n
    """

    def __init__(self, file_location, num_scores):
        self.file_location = file_location

        # Write the header to file.
        with open(self.file_location, 'w') as csvFile:
            writer = csv.writer(csvFile)
            csv_header = ['generation', 'identifier'] + ['score_' + str(i) for i in range(num_scores)] + ['final_score']
            writer.writerow(csv_header)

    def write_scores(self, generation, score_table):
        """ Writes the given scores to file, score table contains a dictionary with identifiers as keys and a list
        of scores
        """

        with open(self.file_location, 'a') as csvFile:
            writer = csv.writer(csvFile)

            csv_lines = [[generation, identifier] + scores for identifier, scores in iteritems(score_table) ]
            writer.writerows(csv_lines)






