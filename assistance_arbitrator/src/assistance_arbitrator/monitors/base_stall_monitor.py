#!/usr/bin/env python
# Monitor the commands sent to the root and the resulting movement from the
# robot

from __future__ import print_function, division

import numpy as np

from threading import Lock

import rospy

from assistance_msgs.msg import MonitorMetadata
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

from assistance_arbitrator.monitoring import AbstractFaultMonitor


# The class definition

class BaseStallMonitor(AbstractFaultMonitor):
    """
    Monitor the commands sent to the base and compare against the actual
    movement of the base. If there is a disconnect, flag the error
    """

    BASE_COMMAND_TOPIC = "/cmd_vel"
    ODOM_TOPIC = "/odom"
    BASE_STALL_MONITOR_EVENT_NAME = "base_stall_update"

    ZERO_VELOCITY_TOLERANCE = 0.009  # A value for velocity in odom signifying 0
    DETECTION_WAIT_DURATION = rospy.Duration(30.0)  # Number of seconds to wait before considering the robot stalled

    def __init__(self):
        super(BaseStallMonitor, self).__init__()
        self.set_metadata(topics=[BaseStallMonitor.BASE_COMMAND_TOPIC, BaseStallMonitor.ODOM_TOPIC])

        # Variables to help flag a stall
        self._last_cmd_vel = None
        self._cmd_lock = Lock()

        self.base_is_stalled = False
        self._last_stall_detection = None
        self._last_stall_detection_published = False

        # Setup the subscribers
        self._base_cmd_sub = rospy.Subscriber(
            BaseStallMonitor.BASE_COMMAND_TOPIC,
            Twist,
            self._on_base_cmd
        )
        self._odom_sub = rospy.Subscriber(
            BaseStallMonitor.ODOM_TOPIC,
            Odometry,
            self._on_odom
        )

    def _on_base_cmd(self, cmd_msg):
        with self._cmd_lock:
            self._last_cmd_vel = cmd_msg

    def _on_odom(self, odom_msg):
        with self._cmd_lock:
            if self._last_cmd_vel is None:
                return

            cmd_vel_zero = np.isclose(
                np.linalg.norm([self._last_cmd_vel.linear.x, self._last_cmd_vel.linear.y, self._last_cmd_vel.linear.z]),
                0.0,
                atol=BaseStallMonitor.ZERO_VELOCITY_TOLERANCE
            )
            cmd_vel_zero = cmd_vel_zero and np.isclose(
                np.linalg.norm([self._last_cmd_vel.angular.x, self._last_cmd_vel.angular.y, self._last_cmd_vel.angular.z]),
                0.0,
                atol=BaseStallMonitor.ZERO_VELOCITY_TOLERANCE
            )

        # If the commanded velocity is 0, then we don't need to do anything.
        # Make sure to reset all the detection flags too
        if cmd_vel_zero:
            self.base_is_stalled = False
            if self._last_stall_detection_published:
                self.update_trace(
                    BaseStallMonitor.BASE_STALL_MONITOR_EVENT_NAME,
                    self.base_is_stalled,
                    { 'base_is_stalled': self.base_is_stalled }
                )

            self._last_stall_detection = None
            self._last_stall_detection_published = False
            return

        # Check to see if the odometry is reporting a zero
        odom_vel = odom_msg.twist.twist
        odom_vel_zero = np.isclose(
            np.linalg.norm([odom_vel.linear.x, odom_vel.linear.y, odom_vel.linear.z]),
            0.0,
            atol=BaseStallMonitor.ZERO_VELOCITY_TOLERANCE
        )
        odom_vel_zero = odom_vel_zero and np.isclose(
            np.linalg.norm([odom_vel.angular.x, odom_vel.angular.y, odom_vel.angular.z]),
            0.0,
            atol=BaseStallMonitor.ZERO_VELOCITY_TOLERANCE
        )

        # If the odom velocity is non 0, then there is nothing to worry about.
        # Reset everything and return
        if not odom_vel_zero:
            self.base_is_stalled = False
            if self._last_stall_detection_published:
                self.update_trace(
                    BaseStallMonitor.BASE_STALL_MONITOR_EVENT_NAME,
                    self.base_is_stalled,
                    { 'base_is_stalled': self.base_is_stalled }
                )

            self._last_stall_detection = None
            self._last_stall_detection_published = False
            return

        # Now for the tree of cases, one of which leads to an event being sent
        # to the trace. We don't want to spam the trace at 100 Hz.
        self.base_is_stalled = True
        if self._last_stall_detection is None:
            self._last_stall_detection = rospy.Time.now()
        elif rospy.Time.now() >= self._last_stall_detection + BaseStallMonitor.DETECTION_WAIT_DURATION \
                and not self._last_stall_detection_published:
            rospy.loginfo("Detected a stalled robot base")
            self.update_trace(
                BaseStallMonitor.BASE_STALL_MONITOR_EVENT_NAME,
                self.base_is_stalled,
                { 'base_is_stalled': self.base_is_stalled }
            )
            self._last_stall_detection_published = True


# When running the monitor in standalone mode
if __name__ == '__main__':
    rospy.init_node('base_stall_monitor')
    monitor = BaseStallMonitor()
    rospy.spin()
