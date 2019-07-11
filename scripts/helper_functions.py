import rospy

def get_available_namespaces():
    """ This function returns the namespaces of the simulators that are currently available."""

    search_string = 'simreport_topic'
    score_topics = [topic[0] for topic in rospy.get_published_topics() if search_string in topic[0]]
    name_spaces = [topic.replace(search_string, '') for topic in score_topics]

    return name_spaces