#!/usr/bin/env python
# The node for the action server that executes the task plan

import rospy

from predefined_strategy.server import PredefinedRecoveryServer


def main():
    rospy.init_node('predefined_strategy')
    server = PredefinedRecoveryServer()
    server.start()
    rospy.spin()


if __name__ == '__main__':
    main()
