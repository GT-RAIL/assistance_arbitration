#!/usr/bin/env python
# Monitor and process the execution trace

from __future__ import print_function, division

import os
import collections
import numpy as np

from ruamel.yaml import YAML

import rospy
import rospkg

from actionlib_msgs.msg import GoalStatus
from assistance_msgs.msg import (ExecutionEvent, TaskStepMetadata,
                                 MonitorMetadata, BeliefMetadata, BeliefKeys)

from task_executor.actions import action_names


# Helper functions and classes

yaml = YAML(typ='safe')


class classproperty(property):
    def __get__(self, cls, owner):
        return classmethod(self.fget).__get__(None, owner)()


EVENT_TYPE_DICT = { getattr(ExecutionEvent, x): x for x in dir(ExecutionEvent) if x.isupper() }


EVENT_STATUS_DICT = {
    GoalStatus.ACTIVE: 0,
    GoalStatus.SUCCEEDED: 1,
    GoalStatus.ABORTED: -1,
}


# The tracer object that collects the trace

class ExecutionTracer(object):
    """
    This class monitors the execution trace messages and compiles the data into
    a events trace stream
    """

    EXECUTION_TRACE_TOPIC = '/execution_monitor/trace'
    MAX_TRACE_LENGTH = 9999

    # Stub event type definition
    TIME_EVENT = 255

    # Events to ignore
    EXCLUDE_BELIEF_EVENTS = set([])
    EXCLUDE_MONITOR_EVENTS = set([
        'task_action_recv_result',
        'task_action_send_goal',
        'task_action_cancel',
        'task_service_called',
        'task_topic_message',
        'task_topic_published',
    ])
    EXCLUDE_TASK_STEP_EVENTS = set([])

    # Events to include. These are a list because they need to be ordered
    INCLUDE_BELIEF_EVENTS = []
    INCLUDE_MONITOR_EVENTS = [
        'breaker_state_update',
        'wifi_update',
    ]
    INCLUDE_TASK_STEP_EVENTS = []
    UNKNOWN_TASK_NAME = '<unknown>'
    DEFAULT_TASKS_PARAM = '/task_executor/tasks'

    # This is a vector, used to index into rows
    _trace_types = None
    _trace_types_idx = None

    # Flags for generating the vector of trace types
    AUTO_INCLUDE_BELIEF_EVENTS = True   # Populated from BeliefKeys
    AUTO_INCLUDE_TASK_EVENTS = True     # Populated from default_actions_dict and tasks.yaml

    # Flags for operating on the vector of trace types
    INCLUDE_UNKNOWN_TASK_EVENTS = True

    def __init__(self, start_time=None, create_parsed_events=False):
        start_time = start_time or rospy.Time.now()
        self._create_parsed_events = create_parsed_events

        # Book-keeping variables to keep track of the trace state
        self.full_trace = collections.deque(maxlen=ExecutionTracer.MAX_TRACE_LENGTH)
        if self._create_parsed_events:
            self.parsed_trace = collections.deque(maxlen=ExecutionTracer.MAX_TRACE_LENGTH)
        else:
            self.parsed_trace = None
        self._trace = np.ones((len(ExecutionTracer.trace_types), ExecutionTracer.MAX_TRACE_LENGTH,), dtype=np.float) * np.nan
        self._should_trace = False  # Variable that determines whether to trace

        # Book-keeping variables to help with the trace parsing
        self._tasks_to_reset = set()

        # Initialize the trace
        self.initialize_trace(start_time)

        # Setup the subscriber to track the trace
        self._trace_sub = rospy.Subscriber(
            ExecutionTracer.EXECUTION_TRACE_TOPIC,
            ExecutionEvent,
            self.update_trace
        )

    @classproperty
    def trace_types(cls):
        if cls._trace_types is None:
            # Auto generate the belief events if the flag is set
            if cls.AUTO_INCLUDE_BELIEF_EVENTS:
                cls.INCLUDE_BELIEF_EVENTS += [
                    getattr(BeliefKeys, x) for x in sorted(dir(BeliefKeys))
                    if (
                        x.isupper()
                        and getattr(BeliefKeys, x) not in cls.EXCLUDE_BELIEF_EVENTS
                        and getattr(BeliefKeys, x) not in cls.INCLUDE_BELIEF_EVENTS
                    )
                ]

            # Auto generate task events if the flag is set
            if cls.AUTO_INCLUDE_TASK_EVENTS:

                # First fetch all the defined actions
                actions = set(action_names)

                # Then fetch all the defined tasks
                tasks = rospy.get_param(ExecutionTracer.DEFAULT_TASKS_PARAM, {})
                tasks = set(tasks.keys())

                # Make sure that there are no duplicates
                assert len(actions & tasks) == 0, \
                    "Rename Required: {} are defined as both tasks and actions".format((actions & tasks))

                # Finally, create the list of events to include
                cls.INCLUDE_TASK_STEP_EVENTS += [
                    x for x in sorted((actions | tasks))
                    if (x not in cls.EXCLUDE_TASK_STEP_EVENTS and x not in cls.INCLUDE_TASK_STEP_EVENTS)
                ]

            cls._trace_types = (
                [(ExecutionTracer.TIME_EVENT, 'time')]
                + [(ExecutionEvent.BELIEF_EVENT, name) for name in cls.INCLUDE_BELIEF_EVENTS]
                + [(ExecutionEvent.MONITOR_EVENT, name) for name in cls.INCLUDE_MONITOR_EVENTS]
                + [(ExecutionEvent.TASK_STEP_EVENT, name) for name in cls.INCLUDE_TASK_STEP_EVENTS]
            )

            # If unknown task events should be included (which includes the
            # excluded events), then create a placeholder for them
            if cls.INCLUDE_UNKNOWN_TASK_EVENTS:
                cls.INCLUDE_TASK_STEP_EVENTS.append(ExecutionTracer.UNKNOWN_TASK_NAME)
                cls._trace_types.append(
                    (ExecutionEvent.TASK_STEP_EVENT, ExecutionTracer.UNKNOWN_TASK_NAME)
                )

        return cls._trace_types

    @classproperty
    def trace_types_idx(cls):
        if cls._trace_types_idx is None:
            cls._trace_types_idx = { x: i for i, x in enumerate(ExecutionTracer.trace_types) }
        return cls._trace_types_idx

    @staticmethod
    def trace_idx_by_type(trace_type):
        if trace_type == ExecutionEvent.BELIEF_EVENT:
            trace_names = ExecutionTracer.INCLUDE_BELIEF_EVENTS
        elif trace_type == ExecutionEvent.MONITOR_EVENT:
            trace_names = ExecutionTracer.INCLUDE_MONITOR_EVENTS
        elif trace_type == ExecutionEvent.TASK_STEP_EVENT:
            trace_names = ExecutionTracer.INCLUDE_TASK_STEP_EVENTS
        else:
            raise ValueError("Unknown trace type: {}".format(trace_type))

        return [ExecutionTracer.trace_types_idx[(trace_type, n,)] for n in trace_names]

    @staticmethod
    def get_event_type(event):
        return EVENT_TYPE_DICT.get(event.type)

    @staticmethod
    def discretize_task_step_status(status):
        return EVENT_STATUS_DICT.get(status, np.nan)

    @property
    def num_events(self):
        return len(self.full_trace)

    @property
    def trace(self):
        return self._trace[:, :self.num_events]

    def start(self):
        self._should_trace = True

    def stop(self):
        self._should_trace = False

    def initialize_trace(self, start_time):
        """Initialize the first trace event"""
        event = ExecutionEvent(stamp=start_time)
        self.full_trace.append(event)
        if self._create_parsed_events:
            self.parsed_trace.append(self._get_parsed_trace_from_event(event))

        self._trace[0, 0] = start_time.to_time()
        for idx, trace_spec in enumerate(ExecutionTracer.trace_types):
            if trace_spec[0] == ExecutionEvent.MONITOR_EVENT:
                self._trace[idx, 0] = MonitorMetadata.NOMINAL

    def exclude_from_trace(self, msg):
        """Check to see if the msg should be excluded from the trace"""
        # We want to exclude if the tracer hasn't been started
        if not self._should_trace:
            return True

        # We want to ignore ROSGRAPH events
        if msg.type == ExecutionEvent.ROSGRAPH_EVENT:
            return True

        # Make sure to exclude the subevents that we don't care about
        if msg.name in ExecutionTracer.EXCLUDE_BELIEF_EVENTS and msg.type == ExecutionEvent.BELIEF_EVENT:
            return True

        if msg.name in ExecutionTracer.EXCLUDE_MONITOR_EVENTS and msg.type == ExecutionEvent.MONITOR_EVENT:
            return True

        # Excluded task events are counted as unknowns
        if not ExecutionTracer.INCLUDE_UNKNOWN_TASK_EVENTS \
                and msg.name in ExecutionTracer.EXCLUDE_TASK_STEP_EVENTS \
                and msg.type == ExecutionEvent.TASK_STEP_EVENT:
            return True

        # Warn if there is an event that isn't a known trace type
        if (msg.type, msg.name) not in ExecutionTracer.trace_types:
            # Check that this is not an unknown task we should in fact include
            if not (
                msg.type == ExecutionEvent.TASK_STEP_EVENT
                and ExecutionTracer.INCLUDE_UNKNOWN_TASK_EVENTS
            ):
                rospy.logwarn("Execution Tracer: Unknown event {} ({})"
                              .format(msg.name, ExecutionTracer.get_event_type(msg)))
                return True

        # All is well, include in the trace
        return False

    def update_trace(self, msg):
        """As messages come in, update the trace"""
        if self.exclude_from_trace(msg):
            if self._should_trace:
                rospy.logwarn("Execution Tracer: Discarding event @ {} of type ({})"
                              .format(msg.stamp, ExecutionTracer.get_event_type(msg)))
            return

        # If this is an unknown task and we need to keep track of unknown tasks,
        # then modify the message and keep track of the event
        if msg.type == ExecutionEvent.TASK_STEP_EVENT \
                and (msg.type, msg.name) not in ExecutionTracer.trace_types \
                and ExecutionTracer.INCLUDE_UNKNOWN_TASK_EVENTS:
            msg.name = ExecutionTracer.UNKNOWN_TASK_NAME

        # Append to the full trace
        num_events = self.num_events  # Keep track of the old num_events
        self.full_trace.append(msg)
        if self._create_parsed_events:
            self.parsed_trace.append(self._get_parsed_trace_from_event(msg))

        # Copy over the previous time-step's trace. Also recycle if the trace
        # is too long
        if num_events == ExecutionTracer.MAX_TRACE_LENGTH:
            rospy.logwarn_once(
                "Execution Tracer: Max trace length, {}, reached. Removing old events".format(
                    ExecutionTracer.MAX_TRACE_LENGTH
                )
            )
            self._trace[:, :ExecutionTracer.MAX_TRACE_LENGTH-1] = self._trace[:, 1:]

        self._trace[:, num_events] = self._trace[:, num_events-1]
        current_evt = self._trace[:, num_events]
        current_evt[0] = msg.stamp.to_time()

        # Update all the tasks that have completed
        for task_spec in self._tasks_to_reset:
            current_evt[ExecutionTracer.trace_types_idx[task_spec]] = np.nan
        self._tasks_to_reset = set()

        # Update the task step according to the incoming data
        if msg.type == ExecutionEvent.BELIEF_EVENT:
            current_evt[ExecutionTracer.trace_types_idx[(msg.type, msg.name,)]] = \
                msg.belief_metadata.value
        elif msg.type == ExecutionEvent.MONITOR_EVENT:
            current_evt[ExecutionTracer.trace_types_idx[(msg.type, msg.name,)]] = \
                msg.monitor_metadata.fault_status
        elif msg.type == ExecutionEvent.TASK_STEP_EVENT:
            # Add the task to the list of tasks completed during this iteration
            if msg.task_step_metadata.status in [GoalStatus.SUCCEEDED, GoalStatus.ABORTED]:
                self._tasks_to_reset.add((msg.type, msg.name,))

            # Get the discretized status: -1, 0, 1
            status = ExecutionTracer.discretize_task_step_status(msg.task_step_metadata.status)
            current_evt[ExecutionTracer.trace_types_idx[(msg.type, msg.name,)]] = status
        else:
            raise Exception("Unrecognized event {} of type {}"
                            .format(msg.name, msg.type))

    def _get_parsed_trace_from_event(self, event):
        parsed_event = { 'time': event.stamp.to_time(),
                         'type': ExecutionTracer.get_event_type(event),
                         'name': event.name, }

        if event.type == ExecutionEvent.BELIEF_EVENT:
            parsed_event['value'] = event.belief_metadata.value
        elif event.type == ExecutionEvent.MONITOR_EVENT:
            parsed_event['value'] = event.monitor_metadata.fault_status
        elif event.type == ExecutionEvent.TASK_STEP_EVENT:
            parsed_event['value'] = ExecutionTracer.discretize_task_step_status(event.task_step_metadata.status)
        else:
            parsed_event['type'] = parsed_event['name'] = parsed_event['value'] = None

        return parsed_event
