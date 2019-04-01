#!/usr/bin/env python
# A dash server along with a ROS node that handles and logs user actions

import rospy

from remote_strategy.controller import RemoteController


def main():
    rospy.init_node('remote_controller')
    controller = RemoteController()
    controller.start()
    # The start actually spins


if __name__ == '__main__':
    main()
