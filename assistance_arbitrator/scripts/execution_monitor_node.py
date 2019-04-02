#!/usr/bin/env python
# The node for the action server that executes the task plan

import rospy

from assistance_arbitrator.execution_monitor import ExecutionMonitor

def main():
    rospy.init_node('execution_monitor')
    server = ExecutionMonitor()
    server.start()
    rospy.spin()


if __name__ == '__main__':
    main()
