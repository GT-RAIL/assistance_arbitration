#!/usr/bin/env python
# Test the interfaces to the various default actions that are defined in this
# package

from __future__ import print_function, division

import unittest

import rospy
import actionlib
import rostest

from std_msgs.msg import String
from std_srvs.srv import Trigger, TriggerResponse
from actionlib.msg import TestAction
from assistance_msgs.msg import ExecuteAction, ExecuteGoal

from task_executor.actions import get_default_actions

# Get the local stub node used for testing
from stub_test_node import StubTestNode


# The unit test class

class TestActions(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        # Start the stub test node
        cls.node = StubTestNode()
        cls.node.start()

        # Get the actions
        cls.actions = get_default_actions()
        cls.actions.init()

    def setUp(self):
        self.node.reset()

    def test_wait(self):
        start_time = rospy.Time.now()
        self.actions.wait(duration=2.0)
        self.assertGreaterEqual(rospy.Time.now(), start_time+rospy.Duration(2.0))


# Set this up as a rostest script
if __name__ == '__main__':
    rospy.init_node('test_actions_node')
    rostest.rosrun("task_executor", "test_actions", TestActions)
