#!/usr/bin/env python
# Send a belief update when the gripper has fully closed

from __future__ import print_function, division

import rospy

from fetch_driver_msgs.msg import GripperState
from assistance_msgs.msg import BeliefKeys

from assistance_arbitrator.monitoring import AbstractBeliefMonitor


# The class definition

class GripperClosedMonitor(AbstractBeliefMonitor):
    """
    Monitor the gripper state and fire a belief update on whether it's fully
    closed or not
    """

    GRIPPER_STATE_TOPIC = "/gripper_state"
    GRIPPER_CLOSED_VALUE = 0.004

    def __init__(self):
        super(GripperClosedMonitor, self).__init__()

        # Setup the subscriber
        self._gripper_state_sub = rospy.Subscriber(
            GripperClosedMonitor.GRIPPER_STATE_TOPIC,
            GripperState,
            self._on_gripper_state
        )

    def _on_gripper_state(self, msg):
        if msg.joints[0].position <= GripperClosedMonitor.GRIPPER_CLOSED_VALUE:
            value = float(True)
        else:
            value = float(False)

        self.update_beliefs({ BeliefKeys.GRIPPER_FULLY_CLOSED: value })


# When running the monitor in standalone mode
if __name__ == '__main__':
    rospy.init_node('gripper_closed_monitor')
    monitor = GripperClosedMonitor()
    rospy.spin()
