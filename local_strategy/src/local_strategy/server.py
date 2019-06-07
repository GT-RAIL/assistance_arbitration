#!/usr/bin/env python
# The main action server that provides local recovery behaviour

from __future__ import print_function, division

import pickle

import rospy
import actionlib

from actionlib_msgs.msg import GoalStatus
from assistance_msgs.msg import (RequestAssistanceAction,
                                 RequestAssistanceResult, InterventionEvent,
                                 InterventionStartEndMetadata)
from power_msgs.srv import BreakerCommand

from task_executor.actions import get_default_actions
from rail_sound_interface import SoundClient

from .dialogue import DialogueManager


# The server performs local behaviours to resume execution after contact with
# local humans

class LocalRecoveryServer(object):
    """
    Given a request for assistance, this class interfaces with the robot's
    look, speech, and point modules to request assistance
    """

    INTERVENTION_TRACE_TOPIC = '/intervention_monitor/trace'

    def __init__(self):
        # Instantiate the action server to perform the recovery
        self._server = actionlib.SimpleActionServer(
            rospy.get_name(),
            RequestAssistanceAction,
            self.execute,
            auto_start=False
        )

        # The intervention trace publisher
        self._trace_pub = rospy.Publisher(
            LocalRecoveryServer.INTERVENTION_TRACE_TOPIC,
            InterventionEvent,
            queue_size=10
        )

        # The actions that we are interested in using
        self.actions = get_default_actions()

        # The dialogue manager
        self.dialogue_manager = DialogueManager()

    def start(self):
        # Initialize the actions
        self.actions.init()

        # Initialize the dialogue manager
        self.dialogue_manager.start()

        # Finally, start our action server to indicate that we're ready
        self._server.start()
        rospy.loginfo("Local strategy node ready...")

    def execute(self, goal):
        """Execute the request for assistance"""
        result = self._server.get_default_result()
        result.stats.request_received = rospy.Time.now()

        rospy.loginfo("Local: Serving Assistance Request for: {} (status - {})"
                      .format(goal.component, goal.component_status))
        goal.context = pickle.loads(goal.context)

        # The actual error recovery mechanism
        # Sad beep first
        self.actions.beep(beep=SoundClient.BEEP_SAD, async=True)

        # First we look for a person.
        for variables in self.actions.find_closest_person.run(timeout=0.0):
            if self._server.is_preempt_requested() or not self._server.is_active():
                self.actions.find_closest_person.stop()

        # If we exited without success, report the failure. Otherwise, save the
        # person that we found
        if self.actions.find_closest_person.is_preempted():
            result.context = pickle.dumps(variables)
            self._server.set_preempted(result)
            return

        if self.actions.find_closest_person.is_aborted():
            result.context = pickle.dumps(variables)
            self._server.set_aborted(result)
            return

        person = variables['person']

        # Show excitement and solicit help from them
        self.actions.beep(beep=SoundClient.BEEP_EXCITED)
        for response in self.dialogue_manager.request_help(person):
            if self._server.is_preempt_requested() or not self._server.is_active():
                self.dialogue_manager.reset_dialogue()
                self._server.set_preempted(result)
                return

        # If the person rejected the request for help, abort
        if not response[DialogueManager.REQUEST_HELP_RESPONSE_KEY]:
            result.context = pickle.dumps({ 'person': person, 'response': response })
            self.dialogue_manager.reset_dialogue()
            self._server.set_aborted(result)
            return

        # If they agree to provide help, then continue. Remember to update the
        # intervention trace
        result.stats.request_acked = rospy.Time.now()
        trace_msg = InterventionEvent(stamp=result.stats.request_acked,
                                      type=InterventionEvent.START_OR_END_EVENT)
        trace_msg.start_end_metadata.status = InterventionStartEndMetadata.START
        trace_msg.start_end_metadata.request = goal
        trace_msg.start_end_metadata.request.context = pickle.dumps(goal.context)
        self._trace_pub.publish(trace_msg)
        for response in self.dialogue_manager.await_help(goal):
            if self._server.is_preempt_requested() or not self._server.is_active():
                self.dialogue_manager.reset_dialogue()
                self._server.set_preempted(result)
                return

        # Return when the request for help is completed, and update the
        # intervention trace
        result.resume_hint = response[DialogueManager.RESUME_HINT_RESPONSE_KEY]
        result.stats.request_complete = rospy.Time.now()
        trace_msg = InterventionEvent(stamp=result.stats.request_complete,
                                      type=InterventionEvent.START_OR_END_EVENT)
        trace_msg.start_end_metadata.status = InterventionStartEndMetadata.END
        trace_msg.start_end_metadata.response = result
        self._trace_pub.publish(trace_msg)
        self._server.set_succeeded(result)

    def stop(self):
        pass
