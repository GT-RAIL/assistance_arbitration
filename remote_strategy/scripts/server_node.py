#!/usr/bin/env python
# The node for the action server that executes the task plan

import rospy

from remote_strategy.server import RemoteRecoveryServer


def main():
    rospy.init_node('remote_strategy')
    server = RemoteRecoveryServer()
    server.start()
    rospy.spin()


if __name__ == '__main__':
    main()
