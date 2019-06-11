#!/usr/bin/env python
# This file helps determine the recovery actions to take based on the request
# for assistance that was sent in the event of a failure

from __future__ import print_function, division

import copy
import pickle

import numpy as np

import rospy

from actionlib_msgs.msg import GoalStatus
from assistance_msgs.msg import (RequestAssistanceResult, ExecuteGoal,
                                 BeliefKeys)
from manipulation_actions.msg import StoreObjectResult, InHandLocalizeResult

from task_executor.actions import get_default_actions
from assistance_msgs import msg_utils


# This class encapsulates the different strategies for recovering from different
# error situations

class RecoveryStrategies(object):
    """
    This class is responsible for determining the recovery task to execute when
    given the request for assistance goal and its corresponding context. The
    primary workhorse of this class is :meth:`get_strategy` which takes in a
    :class:`RequestAssistanceGoal`, and generates an :class:`ExecuteGoal`, a
    :class:`RequestAssistanceResult`, and a ``context`` on how to proceed when
    the recovery execution is complete.
    """

    # Just get all the BeliefKeys
    BELIEF_KEYS = [x for x in dir(BeliefKeys) if x.isupper()]

    # Constant values that can dictate the behaviour of when to apply different
    # recovery behaviours
    MAX_PENULTIMATE_TASK_ABORTS = 7
    MAX_PRIMARY_TASK_ABORTS = 50

    def __init__(self, tasks_config):
        self._tasks_config = tasks_config
        self._actions = get_default_actions()

    def init(self):
        # Initialize the connections of all the actions
        self._actions.init()

    def get_strategy(self, assistance_goal):
        """
        Given an assistance goal, generate an ExecuteGoal that can be used for
        recovery and the corresponding manner of resumption. By default, we
        return a ``None`` ExecuteGoal and a
        :const:`RequestAssistanceResult.NONE`. The former implies that no goal
        should be executed, the latter that the task should be aborted in the
        event of a failure.

        In addition to task specific recoveries that are defined in the various
        ``if-elif-else`` conditions in this method, there are global recovery
        behaviours that apply to prevent infinite loops, for example:

            1. If the number of times the penultimate task in the hierarchy has \
                failed is > :const:`MAX_PENULTIMATE_TASK_ABORTS`, then the \
                recovery is aborted
            2. If the number of times the main task has aborted is > \
                :const:`MAX_PRIMARY_TASK_ABORTS`, then the recovery is aborted

        Args:
            assistance_goal (assistance_msgs/RequestAssistanceGoal) :
                The request for assistance. The context attribute is unpickled

        Returns:
            (tuple):
                - execute_goal (``assistance_msgs/ExecuteGoal``) a task \
                    goal to execute if any. If ``None``, assume there is no \
                    such goal
                - resume_hint (``RequestAssistanceResult.resume_hint``) a \
                    constant value indicating how execution should proceed
                - resume_context (dict) more fine grained control of the \
                    intended resume_hint
        """
        execute_goal = None
        resume_hint = RequestAssistanceResult.RESUME_NONE
        resume_context = { 'resume_hint': resume_hint }

        # If our actions are not initialized, then recovery should fail because
        # this is an unknown scenario
        if not self._actions.initialized:
            rospy.logwarn("Recovery: cannot execute because actions are not initialized")
            return execute_goal, resume_hint, resume_context

        # Get the task beliefs. We don't expect it to fail
        _, beliefs = self._actions.get_beliefs(belief_keys=RecoveryStrategies.BELIEF_KEYS)

        # Get the number of times things have failed
        component_names, num_aborts = msg_utils.get_number_of_component_aborts(assistance_goal.context)

        # Infinite loop checks that were disabled for the competition
        if len(component_names) > 1 and \
                num_aborts[-2] > RecoveryStrategies.MAX_PENULTIMATE_TASK_ABORTS:
            rospy.loginfo("Recovery: task {} has failed more than {} times".format(
                component_names[-2],
                RecoveryStrategies.MAX_PENULTIMATE_TASK_ABORTS
            ))
            return execute_goal, resume_hint, resume_context
        elif num_aborts[0] > RecoveryStrategies.MAX_PRIMARY_TASK_ABORTS:
            rospy.loginfo("Recovery: primary task {} has failed more than {} times".format(
                component_names[0],
                RecoveryStrategies.MAX_PRIMARY_TASK_ABORTS
            ))
            return execute_goal, resume_hint, resume_context

        # Then it's a giant lookup table
        if (
            assistance_goal.component == 'loop_body_test'
            or assistance_goal.component == 'reposition_recovery_test'
        ):
            if assistance_goal.component == 'loop_body_test':
                rospy.loginfo("Recovery: simply continue")
                resume_hint = RequestAssistanceResult.RESUME_CONTINUE
                resume_context = msg_utils.create_continue_result_context(assistance_goal.context)

            elif assistance_goal.component == 'reposition_recovery_test':
                rospy.loginfo("Recovery: reposition the base")
                location = RecoveryStrategies.get_last_goal_location(beliefs)
                assert location is not None, "reposition back to an unknown goal location"
                goal_params = {
                    "origin_move_location": "waypoints.origin_for_" + location,
                    "move_location": "waypoints." + location,
                }
                execute_goal = ExecuteGoal(
                    name="reposition_recovery_task",
                    params=pickle.dumps(goal_params)
                )

        # Return the recovery options
        rospy.loginfo("Recovery:\ngoal: {}\nresume_hint: {}\ncontext: {}".format(
            execute_goal if execute_goal is None else execute_goal.name,
            resume_hint,
            resume_context
        ))
        return execute_goal, resume_hint, resume_context

    @staticmethod
    def check_contradictory_beliefs(beliefs):
        """
        Given a set of beliefs, check if there is any contradiction.
        Currently only checking the contradiction between task and robot belief about its current location.

        Args:
            beliefs (dict) :

        Returns:
            (bool) : True if there is a contradiction, False otherwise
        """
        robot_beliefs = {}
        task_beliefs = {}
        for belief_key in beliefs:
            # ToDo: lower() may not be necessary
            lower_belief_key = belief_key.lower()
            if "task_at_" in lower_belief_key:
                location = lower_belief_key.replace("task_at_", "")
                task_beliefs[location] = beliefs[belief_key]
            if "robot_at_" in lower_belief_key:
                location = lower_belief_key.replace("robot_at_", "")
                robot_beliefs[location] = beliefs[belief_key]
        for location in set(robot_beliefs.keys()) & set(task_beliefs.keys()):
            if robot_beliefs[location] != task_beliefs[location]:
                return True
        return False

    @staticmethod
    def get_last_goal_location(beliefs):
        """
        Given a set of beliefs, check the last location that the task says the
        robot was trying to get to.
        """
        for belief_key in beliefs:
            # ToDo: lower() may not be necessary
            lower_belief_key = belief_key.lower()
            if "task_at_" in lower_belief_key:
                if beliefs[belief_key] == 1:
                    location = lower_belief_key.replace("task_at_", "")
                    return location
        return None
