#!/usr/bin/env python
# The node for the action server that executes the task plan

import rospy

from local_strategy.server import LocalRecoveryServer


def main():
    rospy.init_node('local_strategy')
    server = LocalRecoveryServer()
    server.start()
    rospy.spin()


if __name__ == '__main__':
    main()
