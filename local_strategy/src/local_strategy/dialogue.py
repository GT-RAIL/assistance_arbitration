#!/usr/bin/env python
# The dialogue manager in the local recovery strategy

from __future__ import print_function, division

import pickle
import numpy as np

from threading import Thread, Lock

import rospy
import actionlib

from sound_interface import SoundClient
from task_executor.actions import get_default_actions, \
    ArmAction  # FIXME

from assistance_msgs.msg import RequestAssistanceResult


# Helper functions

def find_value_in_context(key, context):
    """
    Finds and returns the first value for key-value pair with 'key' in the given
    context. If not present, returns None
    """
    while key not in context:
        if context.has_key('context'):
            context = context['context']
        else:
            return None
    return context[key]


# The main dialogue manager class

class DialogueManager(object):
    """
    This class manages dialogue with the person that with we seek to get help
    from. Basically, given the help request, the manager generates target
    speech similar to: "Excuse me. Could you please help me? I am failing to
    complete <component>, which is step <step_num>. I think the cause is
    <cause>."

    Other tasks of the manager:
    - List the other components that might have failed and the likely causes for
        those failures
    - If the person wants the robot to perform autonomous actions, then request
        safety checks before acting.
    - Look out for the keyword "STOP". If found, stop all actuation.

    When the person is done, the manager should also handle the dialog to resume
    execution according to the policies outlined in RequestAssistanceResult
    """

    # Behavioural constants
    POST_LOOK_SPEAK_WAIT = 0.5

    # Return values
    REQUEST_HELP_RESPONSE_KEY = 'agree_to_help'
    RESUME_HINT_RESPONSE_KEY = 'resume_hint'

    # Speech commands
    SPEECH_GREETING = 'GREETING'
    SPEECH_HEAR_CHECK = 'HEAR_CHECK'
    SPEECH_SMALL_TALK = 'SMALL_TALK'
    SPEECH_ASSIST_AGREE = 'ASSIST_AGREE'
    SPEECH_ASSIST_DISAGREE = 'ASSIST_DISAGREE'
    SPEECH_ASSISTANCE_COMPLETE = 'ASSISTANCE_COMPLETE'
    SPEECH_ENABLE_BASE = 'ENABLE_BASE'
    SPEECH_DISABLE_BASE = 'DISABLE_BASE'
    SPEECH_RESUME_NONE = 'RESUME_NONE'
    SPEECH_RESUME_CONTINUE = 'RESUME_CONTINUE'
    SPEECH_RESUME_RETRY = 'RESUME_RETRY'
    SPEECH_RESUME_NEXT = 'RESUME_NEXT'
    SPEECH_RESUME_PREVIOUS = 'RESUME_PREVIOUS'

    # Template texts
    SAY_HELLO = "Hello!"
    SAY_I_CAN_HEAR_YOU = "I can hear you"
    SAY_SMALL_TALK = "I am a rolling stone that gathers no moss"
    SAY_REQUEST_HELP = """
Excuse me, I have encountered an error and need help in my task. Could you
assist me?
    """
    SAY_DID_NOT_UNDERSTAND = "Sorry, I did not understand that"
    SAY_THATS_OK = "That's OK"
    SAY_OK = "OK"
    SAY_THANKS = "Thank you!"
    SAY_PLEASE_WAIT = "Please wait..."
    SAY_INITIAL_REQUEST = """
I am failing to complete {component}, which is in step {step_num} in my task
plan. I think the cause is {cause}. I am disabling my arm joints now.
    """
    SAY_INSTRUCTIONS = """
My arm joints are now pliable. You can move then however you wish to when
helping me. You can also command me to turn off all joints. When you are done,
say "I am done!"
    """
    SAY_HOW_TO_PROCEED = "How should I proceed?"
    SAY_PROCEED_VALID_OPTIONS = """
I understand the phrases: "Retry failed action", "Continue to next
action", "Retry previous action", "Restart Task", and "Stop Executing"
    """
    SAY_BYEBYE = "Bye!"

    def __init__(self):
        # Load the actions that are available to us
        self.actions = get_default_actions()

        # Idle behaviours. These are normally always available, however, they
        # can be locked out during interactions
        self._listen_behaviour_lock = Lock()
        # self._idle_behaviour_thread = Thread(target=self.run_idle_behaviours)

    def start(self):
        # Initialize the actions
        self.actions.init()

        # Start the idle behaviours
        # self._idle_behaviour_thread.start()

    def reset_dialogue(self):
        # Stop any actions that we might have started
        self.actions.look_at_closest_person(enable=False)

    def run_idle_behaviours(self):
        speech_generator = self.actions.listen.run()
        while not rospy.is_shutdown():
            rospy.sleep(0.1)
            if not self._listen_behaviour_lock.acquire(False):
                continue

            # Query the speech recognition interface for speech. If the speech
            # is one of GREETING, HEAR_CHECK, or SMALL_TALK, respond
            # appropriately
            try:
                cmd = speech_generator.next()
            except StopIteration as e:
                cmd = cmd['cmd']
                if cmd == DialogueManager.SPEECH_GREETING:
                    self.actions.speak(text=DialogueManager.SAY_HELLO, affect=SoundClient.AFFECT_HAPPY)
                elif cmd == DialogueManager.SPEECH_HEAR_CHECK:
                    self.actions.speak(text=DialogueManager.SAY_I_CAN_HEAR_YOU, affect=SoundClient.AFFECT_CALM)
                elif cmd == DialogueManager.SPEECH_SMALL_TALK:
                    self.actions.speak(text=DialogueManager.SAY_SMALL_TALK, affect=SoundClient.AFFECT_CALM)
                speech_generator = self.actions.listen.run()

            # Remember to release the lock
            self._listen_behaviour_lock.release()

    def request_help(self, person):
        # Ternary variable. None: unknown, True: will help, False: won't help
        person_will_help = None
        with self._listen_behaviour_lock:
            # Set the person and start looking at them
            self.actions.look_at_closest_person(enable=True, person_id=person.id)

            # Wait a bit, then send out an auditory request for assistance
            start_time = rospy.Time.now()
            while rospy.Time.now() - start_time <= rospy.Duration(DialogueManager.POST_LOOK_SPEAK_WAIT):
                yield {}

            self.actions.speak(text=DialogueManager.SAY_REQUEST_HELP)

            # Wait for a response
            while person_will_help is None:
                for variables in self.actions.listen.run(expected_cmd=[
                    DialogueManager.SPEECH_ASSIST_AGREE,
                    DialogueManager.SPEECH_ASSIST_DISAGREE,
                ]):
                    yield variables

                if self.actions.listen.is_aborted():
                    cmd = variables['received_cmd']
                    rospy.loginfo("Received unexpected cmd: {}".format(cmd))
                    self.actions.speak(text=DialogueManager.SAY_DID_NOT_UNDERSTAND)
                else:
                    cmd = variables['cmd']
                    if cmd == DialogueManager.SPEECH_ASSIST_DISAGREE:
                        self.actions.speak(
                            text="{}. {}".format(
                                DialogueManager.SAY_THATS_OK,
                                DialogueManager.SAY_BYEBYE
                            )
                        )
                        self.reset_dialogue()
                        person_will_help = False
                    else:  # cmd == DialogueManager.SPEECH_ASSIST_AGREE
                        self.actions.speak(text="{}. {}".format(
                            DialogueManager.SAY_THANKS,
                            DialogueManager.SAY_PLEASE_WAIT
                        ))
                        person_will_help = True

        yield { DialogueManager.REQUEST_HELP_RESPONSE_KEY: person_will_help }

    def await_help(self, request):
        # Get the cause dict
        cause_dict = self._get_cause_from_request(request)
        rospy.loginfo("Likely causes of trouble: {}".format(cause_dict))

        resume_hint = None
        with self._listen_behaviour_lock:
            # First speak the cause of the error
            for variables in self.actions.speak.run(
                text=DialogueManager.SAY_INITIAL_REQUEST.format(**cause_dict)
            ):
                yield variables

            # Then let the person know that the robot is movable
            self.actions.toggle_breakers(enable_arm=False)
            rospy.sleep(0.5)
            self.actions.toggle_breakers(enable_arm=True)
            for variables in self.actions.speak.run(
                text=DialogueManager.SAY_INSTRUCTIONS
            ):
                yield variables

            # Wait for an I'm done
            done_flag = False
            while not done_flag:
                for variables in self.actions.listen.run(
                    expected_cmd=[
                        DialogueManager.SPEECH_ASSISTANCE_COMPLETE,
                        DialogueManager.SPEECH_ENABLE_BASE,
                        DialogueManager.SPEECH_DISABLE_BASE,
                    ]
                ):
                    yield variables

                if self.actions.listen.is_aborted():
                    cmd = variables['received_cmd']
                    rospy.loginfo("Received unexpected cmd: {}".format(cmd))
                    self.actions.speak(text=DialogueManager.SAY_DID_NOT_UNDERSTAND)
                else:
                    cmd = variables['cmd']
                    if cmd == DialogueManager.SPEECH_ASSISTANCE_COMPLETE:
                        done_flag = True
                        self.actions.speak(text="{}. {}".format(
                            DialogueManager.SAY_THANKS,
                            DialogueManager.SAY_HOW_TO_PROCEED
                        ))
                    elif cmd == DialogueManager.SPEECH_ENABLE_BASE:
                        self.actions.toggle_breakers(enable_base=True)
                        self.actions.speak(text=DialogueManager.SAY_OK)
                    elif cmd == DialogueManager.SPEECH_DISABLE_BASE:
                        self.actions.toggle_breakers(enable_base=False)
                        self.actions.speak(text=DialogueManager.SAY_OK)

            # Finally, wait for a response
            while resume_hint is None:
                for variables in self.actions.listen.run(expected_cmd=[
                    DialogueManager.SPEECH_RESUME_NONE,
                    DialogueManager.SPEECH_RESUME_CONTINUE,
                    DialogueManager.SPEECH_RESUME_RETRY,
                    DialogueManager.SPEECH_RESUME_NEXT,
                    DialogueManager.SPEECH_RESUME_PREVIOUS,
                ]):
                    yield variables

                if self.actions.listen.is_aborted():
                    cmd = variables['received_cmd']
                    rospy.loginfo("Received unexpected cmd: {}".format(cmd))
                    for variables in self.actions.speak.run(text="{}. {}".format(
                        DialogueManager.SAY_DID_NOT_UNDERSTAND,
                        DialogueManager.SAY_PROCEED_VALID_OPTIONS
                    )):
                        yield variables
                else:
                    resume_hint = variables['cmd']
                    if resume_hint == DialogueManager.SPEECH_RESUME_NONE:
                        resume_hint = RequestAssistanceResult.RESUME_NONE
                    elif resume_hint == DialogueManager.SPEECH_RESUME_CONTINUE:
                        resume_hint = RequestAssistanceResult.RESUME_CONTINUE
                    elif resume_hint == DialogueManager.SPEECH_RESUME_RETRY:
                        resume_hint = RequestAssistanceResult.RESUME_RETRY
                    elif resume_hint == DialogueManager.SPEECH_RESUME_NEXT:
                        resume_hint = RequestAssistanceResult.RESUME_NEXT
                    elif resume_hint == DialogueManager.SPEECH_RESUME_PREVIOUS:
                        resume_hint = RequestAssistanceResult.RESUME_PREVIOUS
                    else:
                        resume_hint = None
                        continue

                    self.actions.speak(text="{}. {}".format(
                        DialogueManager.SAY_THANKS, DialogueManager.SAY_BYEBYE
                    ))
                    self.reset_dialogue()

        self.actions.toggle_breakers()
        yield { DialogueManager.RESUME_HINT_RESPONSE_KEY: resume_hint }

    def _get_cause_from_request(self, request):
        # TODO: This is messy. Fix it after we figure out what's happening here
        # Create a dictionary with keys 'component', 'step_num', and 'cause'
        cause_dict = {
            'component': request.component,
            'step_num': request.context['step_idx'],
        }

        # Now for a giant if-else. TODO: Make this more intelligent and better
        # structured. This should perhaps be where the causal model comes in?
        if request.component == 'find_object':
            found_idx = find_value_in_context('found_idx', request.context)
            if found_idx is not None and found_idx < 0:
                cause_dict['cause'] = "a missing object"
                return cause_dict

        if request.component == 'arm':
            attempt_num = find_value_in_context('attempt_num', request.context)
            if attempt_num is not None and attempt_num + 1 >= ArmAction.MAX_ATTEMPTS:
                cause_dict['cause'] = 'a motion planner failure'
                return cause_dict

        if request.component == 'pick':
            num_grasps = find_value_in_context('num_grasps', request.context)
            grasp_num = find_value_in_context('grasp_num', request.context)
            if num_grasps is not None and grasp_num is not None \
                    and grasp_num + 1 >= num_grasps:
                cause_dict['cause'] = 'all grasp positions failed'
                return cause_dict

        if find_value_in_context('cause', request.context) is not None:
            cause = find_value_in_context('cause', request.context)
            if cause.split()[0].lower() in ['unknown', 'unrecognized', 'invalid']:
                cause_dict['cause'] = 'invalid values in the task'
                return cause_dict

        # Catch all
        cause_dict['cause'] = 'unknown'
        return cause_dict
