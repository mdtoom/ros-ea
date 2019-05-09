#!/usr/bin/env python
import os
import sys

import rospy

if __name__ == '__main__':

    my_argv = rospy.myargv(argv=sys.argv)

    if len(my_argv) != 2:
        print 'Expected one argument, namely a path to an argos file'
        exit(1)

    argos_file = my_argv[1]
    os.system('argos3 -c {0}'.format(argos_file))
