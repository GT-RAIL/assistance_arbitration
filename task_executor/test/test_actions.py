#!/usr/bin/env python
# Test the interfaces to the various default actions that are defined in this
# package

from __future__ import print_function, division

import unittest

import rospy
import actionlib
import rostest

from actionlib_msgs.msg import GoalStatus
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
        status, _ = self.actions.wait(duration=2.0)
        self.assertEqual(status, GoalStatus.SUCCEEDED)
        self.assertGreaterEqual(rospy.Time.now(), start_time+rospy.Duration(2.0))

        start_time = rospy.Time.now()
        for variables in self.actions.wait.run(duration=2.0):
            if rospy.Time.now() > start_time + rospy.Duration(1.0):
                self.actions.wait.stop()
        self.assertEqual(self.actions.wait.status, GoalStatus.PREEMPTED)

    def test_get_string(self):
        self.node.publish()
        status, variables = self.actions.get_string()
        self.assertEqual(status, GoalStatus.SUCCEEDED)
        self.assertEqual(variables['string_data'], self.node.string_value)

        publisher = rospy.Publisher("/test_topic1", String, queue_size=1, latch=True)
        string_value = "what should I name my robot?"
        publisher.publish(string_value)
        status, variables = self.actions.get_string(topic="/test_topic1")
        self.assertEqual(status, GoalStatus.SUCCEEDED)
        self.assertEqual(variables['string_data'], string_value)

    def test_set_string(self):
        string_value = "naming a robot is serious business"
        status, _ = self.actions.set_string(string_data=string_value)
        rospy.sleep(0.5)
        self.assertEqual(status, GoalStatus.SUCCEEDED)
        self.assertEqual(self.node.string_value, string_value)

        status, _ = self.actions.set_string(string_data=string_value, topic="/test_topic2")
        value_returned = rospy.wait_for_message("/test_topic2", String).data
        self.assertEqual(status, GoalStatus.SUCCEEDED)
        self.assertEqual(value_returned, string_value)

    def test_trigger(self):
        status, variables = self.actions.trigger()
        self.assertEqual(status, GoalStatus.SUCCEEDED)
        self.assertTrue(variables['success'])
        self.assertEqual(self.node.trigger_count, 1)

        self.node.trigger_enabled = False
        status, variables = self.actions.trigger()
        self.assertEqual(status, GoalStatus.SUCCEEDED)
        self.assertFalse(variables['success'])
        self.assertEqual(self.node.trigger_count, 1)

    def test_integers(self):
        status, variables = self.actions.integers(input_value=3)
        self.assertEqual(status, GoalStatus.SUCCEEDED)
        self.assertEqual(variables['output_value'], 2)

        start_time = rospy.Time.now()
        for variables in self.actions.integers.run(input_value=6):
            if rospy.Time.now() > start_time + rospy.Duration(self.node.integer_wait-1):
                self.actions.integers.stop()
        self.assertEqual(self.actions.integers.status, GoalStatus.PREEMPTED)

        self.node.integer_enabled = False
        status, variables = self.actions.integers(input_value=10)
        self.assertEqual(status, GoalStatus.ABORTED)
        self.assertListEqual(
            sorted(variables.keys()),
            ['action', 'goal', 'num_aborts', 'result', 'status']
        )


# Set this up as a rostest script
if __name__ == '__main__':
    rospy.init_node('test_actions_node')
    rostest.rosrun("task_executor", "test_actions", TestActions)
