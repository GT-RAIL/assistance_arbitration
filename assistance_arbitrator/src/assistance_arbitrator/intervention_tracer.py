#!/usr/bin/env python
# Monitor and process the execution trace

from __future__ import print_function, division

import sys
import pickle
import importlib
import collections

import numpy as np

import rospy

from assistance_msgs.msg import (RequestAssistanceActionGoal,
                                 RequestAssistanceResult, InterventionEvent,
                                 InterventionHypothesisMetadata,
                                 InterventionActionMetadata,
                                 InterventionStartEndMetadata, ExecutionEvent)

from assistance_msgs import msg_utils
from assistance_arbitrator.execution_tracer import ExecutionTracer, classproperty

# Import isolation
try:
    import isolation
except ImportError as e:
    sys.path.append('/home/banerjs/Libraries/RAIL/codebase/banerjs/isolation/models')
    import isolation
from isolation.data.annotations import Annotations


# Helper functions and classes

EVENT_TYPE_DICT = { getattr(InterventionEvent, x): x for x in dir(InterventionEvent) if x.isupper() }
for _x in dir(InterventionStartEndMetadata):
    if _x.isupper():
        EVENT_TYPE_DICT[(InterventionEvent.START_OR_END_EVENT, getattr(InterventionStartEndMetadata, _x))] = _x
for _x in dir(InterventionHypothesisMetadata):
    if _x.isupper():
        EVENT_TYPE_DICT[(InterventionEvent.HYPOTHESIS_EVENT, getattr(InterventionHypothesisMetadata, _x))] = _x


RESUME_HINT_DICT = { getattr(RequestAssistanceResult, x): x for x in dir(RequestAssistanceResult) if x.isupper() }


HYPOTHESIS_STATUS_DICT = {
    getattr(InterventionHypothesisMetadata, x): x for x in dir(InterventionHypothesisMetadata) if x.isupper()
}


# The tracer class

class InterventionTracer(object):
    """
    Monitors the actions taken during an intervention and logs them to enable
    continual learning of recovery actions. This class functions similarly to
    the ExecutionTracer class, which monitors the robot execution trace.
    """

    INTERVENTION_TRACE_TOPIC = '/intervention_monitor/trace'
    MAX_TRACE_LENGTH = 999

    # Stub event type definition
    TIME_EVENT = ExecutionTracer.TIME_EVENT

    # The expected events to appear on this stream. Most are autopopulated if
    # AUTO_INCLUDE_* flags are turned on
    EXCLUDE_HYPOTHESIS_EVENTS = set([])
    INCLUDE_HYPOTHESIS_EVENTS = []

    EXCLUDE_INT_ACTION_EVENTS = set([])
    INCLUDE_INT_ACTION_EVENTS = []

    EXCLUDE_TASK_STEP_EVENTS = set([])
    INCLUDE_TASK_STEP_EVENTS = []

    # Flags to autopopulate the types of events to include or exclude
    AUTO_INCLUDE_HYPOTHESIS_EVENTS = True   # Populated from Annotations
    AUTO_INCLUDE_INT_ACTION_EVENTS = True       # Populated from InterventionActions
    AUTO_INCLUDE_TASK_STEP_EVENTS = True        # Populated from ExecutionTracer.INCLUDE_TASK_STEP_EVENTS

    DEFAULT_INTERVENTION_ACTIONS_PARAM = '/arbitrator/intervention_actions'
    DEFAULT_INTERVENTION_ACTIONS_PARAM_VALUE = "assistance_msgs.msg.InterventionActions"

    FAILED_COMPONENT_TRACE_NAME = '<component>'
    START_END_FLAG_TRACE_NAME = '<start_end>'

    # Ultimately, from the above these classproperties are populated
    _trace_types = None
    _trace_types_idx = None

    def __init__(self, create_parsed_events=False):
        self._create_parsed_events = create_parsed_events

        # Book-keeping variables to keep track of the intervention events
        self.full_traces = []
        self.parsed_traces = []
        self._traces = []
        self._should_trace = False

        # The subscriber to track the trace
        self._trace_sub = rospy.Subscriber(
            InterventionTracer.INTERVENTION_TRACE_TOPIC,
            InterventionEvent,
            self.update_trace
        )

    @classproperty
    def trace_types(cls):
        if cls._trace_types is None:
            # Auto generate the hypothesis events if the flag is set
            if cls.AUTO_INCLUDE_HYPOTHESIS_EVENTS:
                cls.INCLUDE_HYPOTHESIS_EVENTS += [
                    x['value'] for x in Annotations.RESULT_OPTIONS[1:]
                    if (
                        x['value'] not in cls.EXCLUDE_HYPOTHESIS_EVENTS
                        and x['value'] not in cls.INCLUDE_HYPOTHESIS_EVENTS
                    )
                ]

            # Auto generate the action events if the flag is set
            if cls.AUTO_INCLUDE_INT_ACTION_EVENTS:
                actions_param = rospy.get_param(
                    InterventionTracer.DEFAULT_INTERVENTION_ACTIONS_PARAM,
                    InterventionTracer.DEFAULT_INTERVENTION_ACTIONS_PARAM_VALUE
                )
                actions_module, actions_class = actions_param.rsplit('.', 1)
                actions_module = importlib.import_module(actions_module)
                actions_class = getattr(actions_module, actions_class)

                cls.INCLUDE_INT_ACTION_EVENTS += [
                    getattr(actions_class, x) for x in sorted(dir(actions_class))
                    if (
                        x.isupper()
                        and getattr(actions_class, x) not in cls.EXCLUDE_INT_ACTION_EVENTS
                        and getattr(actions_class, x) not in cls.INCLUDE_INT_ACTION_EVENTS
                    )
                ]

            # Auto generate the start events if the flag is set
            if cls.AUTO_INCLUDE_TASK_STEP_EVENTS:
                # Initialize the ExecutionTracer trace types
                _ = ExecutionTracer.trace_types

                cls.INCLUDE_TASK_STEP_EVENTS += [
                    x for x in ExecutionTracer.INCLUDE_TASK_STEP_EVENTS
                    if (
                        x not in cls.EXCLUDE_TASK_STEP_EVENTS
                        and x not in cls.INCLUDE_TASK_STEP_EVENTS
                    )
                ]

            # Sanity check that the component name and the start/end flag will
            # not be in conflict
            assert InterventionTracer.FAILED_COMPONENT_TRACE_NAME not in cls.INCLUDE_TASK_STEP_EVENTS, \
                "Rename Required: {} cannot be the name of a task or action".format(
                    InterventionTracer.FAILED_COMPONENT_TRACE_NAME
                )
            assert InterventionTracer.START_END_FLAG_TRACE_NAME not in cls.INCLUDE_TASK_STEP_EVENTS, \
                "Rename Required: {} cannot be the name of a task or action".format(
                    InterventionTracer.START_END_FLAG_TRACE_NAME
                )

            cls._trace_types = (
                [(InterventionTracer.TIME_EVENT, None, 'time')]
                + [(InterventionEvent.HYPOTHESIS_EVENT, x) for x in cls.INCLUDE_HYPOTHESIS_EVENTS]
                + [(InterventionEvent.ACTION_EVENT, x) for x in cls.INCLUDE_INT_ACTION_EVENTS]
                + [(InterventionEvent.START_OR_END_EVENT, x) for x in cls.INCLUDE_TASK_STEP_EVENTS]
                + [(InterventionEvent.START_OR_END_EVENT, InterventionTracer.FAILED_COMPONENT_TRACE_NAME),
                   (InterventionEvent.START_OR_END_EVENT, InterventionTracer.START_END_FLAG_TRACE_NAME)]
            )

        return cls._trace_types

    @classproperty
    def trace_types_idx(cls):
        if cls._trace_types_idx is None:
            cls._trace_types_idx = { x: i for i, x in enumerate(InterventionTracer.trace_types) }
        return cls._trace_types_idx

    @staticmethod
    def trace_idx_by_type(trace_type):
        if trace_type == InterventionEvent.HYPOTHESIS_EVENT:
            trace_names = InterventionTracer.INCLUDE_HYPOTHESIS_EVENTS
        elif trace_type == InterventionEvent.ACTION_EVENT:
            trace_names = InterventionTracer.INCLUDE_INT_ACTION_EVENTS
        elif trace_type == InterventionEvent.START_OR_END_EVENT:
            trace_names = InterventionTracer.INCLUDE_TASK_STEP_EVENTS
        elif trace_type in [InterventionTracer.FAILED_COMPONENT_TRACE_NAME, InterventionTracer.START_END_FLAG_TRACE_NAME]:
            trace_names = [trace_type]
            trace_type = InterventionEvent.START_OR_END_EVENT
        else:
            raise ValueError("Unknown trace type: {}".format(trace_type))

        return [InterventionTracer.trace_types_idx[(trace_type, n,)] for n in trace_names]

    @staticmethod
    def get_event_type(event, complete=False):
        event_type = None
        if event.type == InterventionEvent.START_OR_END_EVENT:
            event_type = (event.type, event.start_end_metadata.status,)
        elif event.type == InterventionEvent.HYPOTHESIS_EVENT and complete:
            event_type = (event.type, event.hypothesis_metadata.status,)
        else:
            event_type = event.type
        return EVENT_TYPE_DICT.get(event_type)

    @staticmethod
    def get_hypothesis_status(event):
        return HYPOTHESIS_STATUS_DICT.get(event.hypothesis_metadata.status)

    @staticmethod
    def get_resume_strategy(event):
        return RESUME_HINT_DICT.get(event.start_end_metadata.response.resume_hint)

    @property
    def num_interventions(self):
        return len(self.full_traces)

    @property
    def last_event(self):
        if self.num_interventions == 0:
            return None
        return self.full_traces[-1][-1]

    def num_events(self, trace_idx):
        if len(self.full_traces) == 0:
            return None

        return len(self.full_traces[trace_idx])

    def trace(self, trace_idx):
        return self._traces[trace_idx][:, :self.num_events(trace_idx)]

    @property
    def traces(self):
        return [self.trace(idx) for idx in xrange(self.num_interventions)]

    def start(self):
        self._should_trace = True

    def stop(self):
        self._should_trace = False

    def initialize_traces(self):
        self.full_traces.append(collections.deque(maxlen=InterventionTracer.MAX_TRACE_LENGTH))
        if self._create_parsed_events:
            self.parsed_traces.append(collections.deque(maxlen=InterventionTracer.MAX_TRACE_LENGTH))
        self._traces.append(
            np.ones((len(InterventionTracer.trace_types), InterventionTracer.MAX_TRACE_LENGTH,), dtype=np.float) * np.nan
        )
        self._traces[-1][InterventionTracer.trace_idx_by_type(InterventionEvent.HYPOTHESIS_EVENT), 0] = \
            InterventionHypothesisMetadata.ABSENT

    def exclude_from_trace(self, msg):
        """Check to see if the message should be excluded from the trace"""
        if not self._should_trace:
            return True

        # Warn if this is an event type that is not recognized
        if msg.type not in [InterventionEvent.START_OR_END_EVENT,
                            InterventionEvent.HYPOTHESIS_EVENT,
                            InterventionEvent.ACTION_EVENT]:
            return True

        # Check against the include/exclude sanity checks
        if (
            msg.type == InterventionEvent.START_OR_END_EVENT
            and msg.start_end_metadata.status not in [
                InterventionStartEndMetadata.START,
                InterventionStartEndMetadata.END,
            ]
        ):
            return True

        if (
            msg.type == InterventionEvent.START_OR_END_EVENT
            and msg.start_end_metadata.status == InterventionStartEndMetadata.START
            and (msg.type, msg.start_end_metadata.request.component,)
                not in InterventionTracer.trace_types_idx
            and not ExecutionTracer.INCLUDE_UNKNOWN_TASK_EVENTS
        ):
            return True


        if (
            msg.type == InterventionEvent.START_OR_END_EVENT
            and msg.start_end_metadata.status == InterventionStartEndMetadata.END
            and (msg.type, msg_utils.get_final_resume_context(msg.start_end_metadata.response.context).get('task'))
                not in InterventionTracer.trace_types_idx
            and not ExecutionTracer.INCLUDE_UNKNOWN_TASK_EVENTS
        ):
            return True

        if (
            msg.type == InterventionEvent.HYPOTHESIS_EVENT
            and msg.hypothesis_metadata.status not in [
                InterventionHypothesisMetadata.ABSENT,
                InterventionHypothesisMetadata.CONFIRMED,
                InterventionHypothesisMetadata.SUSPECTED,
            ]
        ):
            return True

        if (
            msg.type == InterventionEvent.HYPOTHESIS_EVENT
            and (msg.type, msg.hypothesis_metadata.name)
                not in InterventionTracer.trace_types_idx
        ):
            return True

        if (
            msg.type == InterventionEvent.ACTION_EVENT
            and (msg.type, msg.action_metadata.type) not in InterventionTracer.trace_types_idx
        ):
            return True

        # Finally, check that we are potentially tracking an intervention trace
        if (
            self.last_event is None
            or (self.last_event.type == InterventionEvent.START_OR_END_EVENT
                and self.last_event.start_end_metadata.status == InterventionStartEndMetadata.END)
        ) and (
            msg.type != InterventionEvent.START_OR_END_EVENT
            or msg.start_end_metadata.status != InterventionStartEndMetadata.START
        ):
            rospy.logwarn("Intervention Tracer: Attempting to add to non-existent intervention")
            return True

        if (
            self.last_event is not None
            and (self.last_event.type != InterventionEvent.START_OR_END_EVENT
                or self.last_event.start_end_metadata.status != InterventionStartEndMetadata.END)
        ) and (
            msg.type == InterventionEvent.START_OR_END_EVENT
            and msg.start_end_metadata.status == InterventionStartEndMetadata.START
        ):
            rospy.logwarn("Intervention Tracer: Attempting to start in the middle of another intervention")
            return True

        # All is well, include in the trace
        return False

    def update_trace(self, msg):
        # Unpickle the context if this a start or end event. We will need this
        # later in the processing pipeline
        if msg.type == InterventionEvent.START_OR_END_EVENT:
            if msg.start_end_metadata.status == InterventionStartEndMetadata.START:
                msg.start_end_metadata.request.context = \
                    msg_utils.unpickle_context(msg.start_end_metadata.request.context)
            elif msg.start_end_metadata.status == InterventionStartEndMetadata.END:
                msg.start_end_metadata.response.context = \
                    msg_utils.unpickle_context(msg.start_end_metadata.response.context)

        # Check to see if this message is valid
        if self.exclude_from_trace(msg):
            rospy.logwarn("Intervention Tracer: Discarding event @ {} of type ({})"
                          .format(msg.stamp, InterventionTracer.get_event_type(msg, complete=True)))
            return

        # If the trace is too long, warn and stop updating the arrays
        num_events = self.num_events(-1)
        if num_events is not None and num_events >= InterventionTracer.MAX_TRACE_LENGTH:
            rospy.logwarn_once(
                "Intervention Tracer: Max trace length, {}, reached. Not tracking".format(
                    InterventionTracer.MAX_TRACE_LENGTH
                )
            )
            return

        # If this is an unknown task failure, and the execution tracer is set to
        # monitor unknown tasks, then update the message
        if (
            msg.type == InterventionEvent.START_OR_END_EVENT
            and msg.start_end_metadata.status == InterventionStartEndMetadata.START
            and (msg.type, msg.start_end_metadata.request.component)
                not in InterventionTracer.trace_types_idx
            and ExecutionTracer.INCLUDE_UNKNOWN_TASK_EVENTS
        ):
            msg.start_end_metadata.request.component = ExecutionTracer.UNKNOWN_TASK_NAME
            context = msg.start_end_metadata.request.context
            while context is not None:
                if context.get('task', context.get('action')) not in InterventionTracer.INCLUDE_TASK_STEP_EVENTS:
                    if context.has_key('task'):
                        context['task'] = ExecutionTracer.UNKNOWN_TASK_NAME
                    elif context.has_key('action'):
                        context['action'] = ExecutionTracer.UNKNOWN_TASK_NAME
                context = context.get('context')
        elif (
            msg.type == InterventionEvent.START_OR_END_EVENT
            and msg.start_end_metadata.status == InterventionStartEndMetadata.END
            and (msg.type, msg_utils.get_final_resume_context(msg.start_end_metadata.response.context).get('task'))
                not in InterventionTracer.trace_types_idx
            and ExecutionTracer.INCLUDE_UNKNOWN_TASK_EVENTS
        ):
            context = msg.start_end_metadata.response.context
            while context is not None and len(context) > 0:
                if context['task'] not in InterventionTracer.INCLUDE_TASK_STEP_EVENTS:
                    context['task'] = ExecutionTracer.UNKNOWN_TASK_NAME
                context = context.get('context')

        # Append the full trace
        if msg.type == InterventionEvent.START_OR_END_EVENT \
                and msg.start_end_metadata.status == InterventionStartEndMetadata.START:
            self.initialize_traces()

        # Add to the trace
        self.full_traces[-1].append(msg)
        if self._create_parsed_events:
            self.parsed_traces[-1].append(self._get_parsed_event_from_event(msg))
        num_events = self.num_events(-1)

        # Copy the previous time stamp over (only for the hypotheses)
        if num_events > 1:
            self._traces[-1][InterventionTracer.trace_idx_by_type(InterventionEvent.HYPOTHESIS_EVENT), num_events-1] = \
                self._traces[-1][InterventionTracer.trace_idx_by_type(InterventionEvent.HYPOTHESIS_EVENT), num_events-2]
        current_event = self._traces[-1][:, num_events-1]
        current_event[0] = msg.stamp.to_time()

        # Based on the type, update the trace
        # First, if it was a start
        if msg.type == InterventionEvent.START_OR_END_EVENT \
                and msg.start_end_metadata.status == InterventionStartEndMetadata.START:
            # First update the tasks in the context
            context = msg.start_end_metadata.request.context
            while context is not None:
                task_name = context.get('task') or context.get('action')
                row = InterventionTracer.trace_types_idx[(msg.type, task_name)]
                current_event[row] = context.get('num_aborts', 0)
                context = context.get('context')

            # Then update the component name and the flag
            row = InterventionTracer.trace_types_idx[(msg.type, InterventionTracer.FAILED_COMPONENT_TRACE_NAME)]
            current_event[row] = InterventionTracer.INCLUDE_TASK_STEP_EVENTS.index(msg.start_end_metadata.request.component)
            row = InterventionTracer.trace_types_idx[(msg.type, InterventionTracer.START_END_FLAG_TRACE_NAME)]
            current_event[row] = msg.start_end_metadata.status

        # Then if it was an end
        elif msg.type == InterventionEvent.START_OR_END_EVENT \
                and msg.start_end_metadata.status == InterventionStartEndMetadata.END:
            # First update the tasks in the context
            context = msg.start_end_metadata.response.context
            while context is not None and len(context) > 0:
                task_name = context['task']
                row = InterventionTracer.trace_types_idx[(msg.type, task_name)]
                resume_hint = context.get('resume_hint', RequestAssistanceResult.RESUME_CONTINUE)
                current_event[row] = resume_hint
                if resume_hint != RequestAssistanceResult.RESUME_CONTINUE:
                    break
                context = context.get('context')

            # Then update the flag. The component name is the last component
            row = InterventionTracer.trace_types_idx[(msg.type, InterventionTracer.FAILED_COMPONENT_TRACE_NAME)]
            current_event[row] = InterventionTracer.INCLUDE_TASK_STEP_EVENTS.index(task_name)
            row = InterventionTracer.trace_types_idx[(msg.type, InterventionTracer.START_END_FLAG_TRACE_NAME)]
            current_event[row] = msg.start_end_metadata.status

        # Then if it was an action
        elif msg.type == InterventionEvent.ACTION_EVENT:
            row = InterventionTracer.trace_types_idx[(msg.type, msg.action_metadata.type,)]
            current_event[row] = ExecutionTracer.discretize_task_step_status(msg.action_metadata.status)

        # Finally, if it was a hypothesis event
        elif msg.type == InterventionEvent.HYPOTHESIS_EVENT:
            row = InterventionTracer.trace_types_idx[(msg.type, msg.hypothesis_metadata.name,)]
            current_event = msg.hypothesis_metadata.status

    def _get_parsed_event_from_event(self, event):
        # Parsed events for interventions
        parsed_event = { 'time': event.stamp.to_time(),
                         'type': InterventionTracer.get_event_type(event) }

        if event.type == InterventionEvent.START_OR_END_EVENT:
            parsed_event['name'] = event.start_end_metadata.status
            parsed_event['value'] = (
                event.start_end_metadata.request.component
                if event.start_end_metadata.status == InterventionStartEndMetadata.START
                else InterventionTracer.get_resume_strategy(event)
            )
        elif event.type == InterventionEvent.HYPOTHESIS_EVENT:
            parsed_event['name'] = event.hypothesis_metadata.name
            parsed_event['value'] = InterventionTracer.get_hypothesis_status(event)
        elif event.type == InterventionEvent.ACTION_EVENT:
            parsed_event['name'] = event.action_metadata.type
            parsed_event['value'] = ExecutionTracer.discretize_task_step_status(event.action_metadata.status)
        else:
            parsed_event['type'] = parsed_event['name'] = parsed_event['value'] = None

        return parsed_event
