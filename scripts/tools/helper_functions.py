import errno
import os

import rospy

def get_available_namespaces():
    """ This function returns the namespaces of the simulators that are currently available."""

    search_string = 'simreport_topic'
    score_topics = [topic[0] for topic in rospy.get_published_topics() if search_string in topic[0]]
    name_spaces = [topic.replace(search_string, '') for topic in score_topics]

    return name_spaces


def create_file(file_name):
    """ This function creates the directories  and the file, such that it can be created afterwards."""
    # Create the directories if they do not exist.
    if not os.path.exists(os.path.dirname(file_name)):
        try:
            os.makedirs(os.path.dirname(file_name))
        except OSError as exc: # Guard against race condition
            if exc.errno != errno.EEXIST:
                raise
    # Create the file.
    with open(file_name, 'w+'):
        pass
