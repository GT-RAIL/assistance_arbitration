#!/usr/bin/env python
# Test the interfaces to the various default actions that are defined in this
# package

from __future__ import print_function, division

import pickle
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

class TestTasks(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        # Start the stub test node
        cls.node = StubTestNode()
        cls.node.start()

        # Create a task client
        cls.client = actionlib.SimpleActionClient("task_executor", ExecuteAction)
        cls.client.wait_for_server()

    def setUp(self):
        self.node.reset()

    def test_choices(self):
        args = {'param1': 1, 'param2': None}
        goal = ExecuteGoal(name='choice_test', params=pickle.dumps(args), no_recoveries=True)
        self.client.send_goal(goal)
        self.client.wait_for_result()
        status = self.client.get_state()
        result = pickle.loads(self.client.get_result().variables)
        self.assertEqual(status, GoalStatus.SUCCEEDED)
        self.assertEqual(result['var1'], 3)

        args = {'param1': 3, 'param2': None}
        goal = ExecuteGoal(name='choice_test', params=pickle.dumps(args), no_recoveries=True)
        self.client.send_goal(goal)
        self.client.wait_for_result()
        status = self.client.get_state()
        result = pickle.loads(self.client.get_result().variables)
        self.assertEqual(status, GoalStatus.SUCCEEDED)
        self.assertEqual(result['var1'], 2)

        args = {'param1': None, 'param2': 2}
        goal = ExecuteGoal(name='choice_test', params=pickle.dumps(args), no_recoveries=True)
        self.client.send_goal(goal)
        self.client.wait_for_result()
        status = self.client.get_state()
        result = pickle.loads(self.client.get_result().variables)
        self.assertEqual(status, GoalStatus.SUCCEEDED)
        self.assertEqual(result['var1'], 1)

    def test_loops(self):
        args = {'num_iterations': 5}
        goal = ExecuteGoal(name='loop_test', params=pickle.dumps(args), no_recoveries=True)
        self.client.send_goal(goal)
        self.client.wait_for_result()
        status = self.client.get_state()
        result = pickle.loads(self.client.get_result().variables)
        self.assertEqual(status, GoalStatus.SUCCEEDED)
        self.assertEqual(self.node.trigger_count, 5)

        args = {'num_iterations': -1}
        goal = ExecuteGoal(name='loop_test', params=pickle.dumps(args), no_recoveries=True)
        self.client.send_goal(goal)
        self.client.wait_for_result()
        status = self.client.get_state()
        result = pickle.loads(self.client.get_result().variables)
        self.assertEqual(status, GoalStatus.SUCCEEDED)
        self.assertEqual(self.node.trigger_count, 5)


# Set this up as a rostest script
if __name__ == '__main__':
    rospy.init_node('test_actions_node')
    rostest.rosrun("task_executor", "test_tasks", TestTasks)
