#!/usr/bin/env python
# Republishes the last move_base goal continuously because RViz doesn't seem to
# latch onto the message

import rospy

from geometry_msgs.msg import PoseStamped


class Republisher(object):
    """
    Republish the goal sent to move_base at the constant frequency
    """

    MOVE_BASE_GOAL_TOPIC = '/move_base/current_goal'
    OUTPUT_TOPIC = '/move_base/republished_goal'
    PUBLISH_FREQUENCY = 2.0

    def __init__(self):
        self.publisher = rospy.Publisher(Republisher.OUTPUT_TOPIC, PoseStamped, queue_size=10)
        self.subscriber = rospy.Subscriber(Republisher.MOVE_BASE_GOAL_TOPIC, PoseStamped, self._on_goal)
        self._current_goal = None

    def _on_goal(self, msg):
        self._current_goal = msg

    def run(self):
        rate = rospy.Rate(Republisher.PUBLISH_FREQUENCY)
        while not rospy.is_shutdown():
            if self._current_goal is not None:
                self.publisher.publish(self._current_goal)
            rate.sleep()


if __name__ == '__main__':
    rospy.init_node('move_goal_republisher')
    republisher = Republisher()
    republisher.run()
