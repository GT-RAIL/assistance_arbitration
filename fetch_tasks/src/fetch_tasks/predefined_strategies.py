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
            assistance_goal.component == 'segment'
            or assistance_goal.component == 'find_grasps'
            or assistance_goal.component == 'find_object'
        ):
            component_idx = component_names.index(assistance_goal.component)
            rospy.loginfo("Recovery: wait and retry")
            self._actions.wait(duration=0.5)

            # If it is any action but segment, retry the whole perception task
            resume_hint = RequestAssistanceResult.RESUME_CONTINUE
            resume_context = msg_utils.create_continue_result_context(assistance_goal.context)
            if 'perceive' in component_names and assistance_goal.component != 'segment':
                resume_context = msg_utils.set_task_hint_in_context(
                    resume_context,
                    'perceive',
                    RequestAssistanceResult.RESUME_RETRY
                )

        elif (
            assistance_goal.component == 'arm'
            or assistance_goal.component == 'pick'
        ):
            rospy.loginfo("Recovery: wait, then clear octomap")
            self._actions.wait(duration=0.5)

            # Try clearing the octomap
            component_idx = component_names.index(assistance_goal.component)

            # If this is a pick, or an arm step in the pick task then also try
            # moving the arm up from the 2nd failure onwards
            if (
                num_aborts[component_idx] >= 2
                and (len(component_names) > 2 and component_names[-2] == 'pick_task')
            ):
                rospy.loginfo("Recovery: also moving 8cm upwards")
                self._actions.arm_cartesian(linear_amount=[0, 0, 0.08])

            # If this the component has failed at least 5 times, then move the
            # head around to clear the octomap
            if num_aborts[component_idx] >= 5:
                execute_goal = ExecuteGoal(name='clear_octomap_task')

        elif assistance_goal.component == 'move':
            rospy.loginfo("Recovery: reposition, then retry move to goal pose")
            component_context = msg_utils.get_final_component_context(assistance_goal.context)
            self._actions.move_planar(angular_amount=np.pi / 10)
            self._actions.wait(duration=0.5)
            self._actions.move_planar(angular_amount=-1 * np.pi / 10)
            self._actions.wait(duration=0.5)
            self._actions.move_planar(linear_amount=-0.1);
            self._actions.wait(duration=0.5)

        elif assistance_goal.component == 'look':
            rospy.loginfo("Recovery: wait and simply retry")
            self._actions.wait(duration=0.5)
            resume_hint = RequestAssistanceResult.RESUME_CONTINUE
            resume_context = msg_utils.create_continue_result_context(assistance_goal.context)

        # Return the recovery options
        rospy.loginfo("Recovery:\ngoal: {}\nresume_hint: {}\ncontext: {}".format(
            execute_goal if execute_goal is None else execute_goal.name,
            resume_hint,
            resume_context
        ))
        return execute_goal, resume_hint, resume_context
