#!/usr/bin/env python
# Run an action that sends an integer, waits for a result, which is also an
# integer, and then returns that value

import rospy
import actionlib

from task_executor.abstract_step import AbstractStep

from actionlib_msgs.msg import GoalStatus
from actionlib.msg import TestAction, TestGoal


class IntegersAction(AbstractStep):
    """
    Send an integer to the action server, and wait for a response, which is
    also an integer. The action server is :const:`INTEGERS_ACTION_SERVER`
    """

    INTEGERS_ACTION_SERVER = "/test_integers"

    def init(self, name):
        self.name = name
        self._integers_client = actionlib.SimpleActionClient(
            IntegersAction.INTEGERS_ACTION_SERVER,
            TestAction
        )

        rospy.loginfo("Connecting to integers...")
        self._integers_client.wait_for_server()
        rospy.loginfo("...integers connected")

    def run(self, input_value):
        """
        The run function for this step

        Args:
            input_value (int) : The integer value to send

        Yields:
            output_value (int) : The returned integer value

        .. seealso::

            :meth:`task_executor.abstract_step.AbstractStep.run`
        """
        rospy.loginfo("Action {}: Input - {}".format(self.name, input_value))

        # Create and send the goal pose
        goal = TestGoal(input_value)
        self._integers_client.send_goal(goal)

        # Yield an empty dict while we're executing
        while self._integers_client.get_state() in AbstractStep.RUNNING_GOAL_STATES:
            yield self.set_running()

        # Wait for a result and yield based on how we exited
        status = self._integers_client.get_state()
        self._integers_client.wait_for_result()
        result = self._integers_client.get_result()

        if status == GoalStatus.SUCCEEDED:
            yield self.set_succeeded(output_value=result.result)
        elif status == GoalStatus.PREEMPTED:
            yield self.set_preempted(
                action=self.name,
                status=status,
                goal=input_value,
                result=result
            )
        else:
            yield self.set_aborted(
                action=self.name,
                status=status,
                goal=input_value,
                result=result
            )

    def stop(self):
        self._integers_client.cancel_goal()
