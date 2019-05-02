#!/usr/bin/env python
from Queue import Queue, Empty

import rospy
from ma_evolution.msg import Score, NEATGenome

genome_queue = Queue()


def callback(data):
    rospy.loginfo(rospy.get_caller_id() + 'I heard %s', data.key)
    genome_queue.put(data)


def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    pub = rospy.Publisher('score_topic', Score, queue_size=10)
    rospy.init_node('executioner', anonymous=True)
    rospy.Subscriber('genome_topic', NEATGenome, callback)

    rospy.loginfo('Started listener with id: %s', rospy.get_caller_id())

    while not rospy.is_shutdown():

        # Decode encoded genome.
        try:
            encoded_genome = genome_queue.get(True, 10.0)
        except Empty:
            rospy.loginfo('No genomes received for 10 seconds.')
            continue

        rospy.loginfo(rospy.get_caller_id() + ' replies to a message from %s', encoded_genome.key)
        score_message = Score(encoded_genome.key, 1.0)
        pub.publish(score_message)


if __name__ == '__main__':
    listener()


