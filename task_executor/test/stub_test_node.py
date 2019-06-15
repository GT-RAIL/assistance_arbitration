#!/usr/bin/env python
# A stub node with action servers, publishers, subscribers, etc. that can be
# used by the different test cases, as needed

from __future__ import print_function, division

import rospy
import actionlib

from std_msgs.msg import String
from std_srvs.srv import Trigger, TriggerResponse
from actionlib.msg import TestAction

# Get the default actions
from task_executor.actions import default_actions_dict


# The stub test node

class StubTestNode(object):
    """
    A stub node with publishers and subscribers that work with the default
    actions in this package
    """

    DEFAULT_STRING_VALUE = "hello world"
    DEFAULT_INTEGERS_WAIT_TIME = 2  # seconds

    def __init__(self):
        # The string publisher and subscriber
        self.string_value = StubTestNode.DEFAULT_STRING_VALUE
        self._string_pub = rospy.Publisher(
            default_actions_dict['get_string'].DEFAULT_STRING_TOPIC,
            String,
            queue_size=1,
            latch=True
        )
        self._string_sub = rospy.Subscriber(
            default_actions_dict['set_string'].DEFAULT_STRING_TOPIC,
            String,
            self._on_string
        )

        # The trigger service
        self.trigger_count = 0
        self.trigger_enabled = True
        self._trigger_service = rospy.Service(
            default_actions_dict['trigger'].TRIGGER_SERVICE_NAME,
            Trigger,
            self._on_trigger
        )

        # The integers action server
        self.integer_func = lambda x: x - 1
        self.integer_enabled = True
        self.integer_wait = StubTestNode.DEFAULT_INTEGERS_WAIT_TIME
        self._integers_server = actionlib.SimpleActionServer(
            default_actions_dict['integers'].INTEGERS_ACTION_SERVER,
            TestAction,
            self._on_integers,
            auto_start=False
        )

    def start(self):
        self._integers_server.start()

    def reset(self):
        self.string_value = StubTestNode.DEFAULT_STRING_VALUE
        self.trigger_count = 0
        self.trigger_enabled = True
        self.integer_func = lambda x: x - 1
        self.integer_enabled = True
        self.integer_wait = StubTestNode.DEFAULT_INTEGERS_WAIT_TIME

    def publish(self):
        self._string_pub.publish(self.string_value)

    def _on_string(self, msg):
        self.string_value = msg.data

    def _on_trigger(self, req):
        if self.trigger_enabled:
            self.trigger_count += 1
        return TriggerResponse(success=self.trigger_enabled)

    def _on_integers(self, goal):
        result = self._integers_server.get_default_result()
        start_time = rospy.Time.now()
        while rospy.Time.now() <= start_time + rospy.Duration(self.integer_wait):
            if self._integers_server.is_preempt_requested():
                self._integers_server.set_preempted(result)
                return

        if self.integer_enabled:
            result.result = self.integer_func(goal.goal)
            self._integers_server.set_succeeded(result)
        else:
            self._integers_server.set_aborted(result)
