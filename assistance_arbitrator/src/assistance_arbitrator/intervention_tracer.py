#!/usr/bin/env python
# Monitor and process the execution trace

from __future__ import print_function, division

import sys
import collections
import numpy as np

import rospy

from assistance_msgs.msg import (RequestAssistanceActionGoal,
                                 RequestAssistanceResult, InterventionEvent,
                                 InterventionHypothesisMetadata,
                                 InterventionActionMetadata,
                                 InterventionStartEndMetadata)

from assistance_arbitrator.execution_tracer import (Tracer as ExecutionTracer,
                                                    classproperty)

# Import isolation
try:
    import isolation
except ImportError as e:
    sys.path.append('/home/banerjs/Libraries/RAIL/codebase/banerjs/isolation/models')
    import isolation
from isolation.data.annotations import Annotations


# Helper functions and classes

EVENT_TYPE_DICT = {
    InterventionEvent.START_OR_END_EVENT: {
        InterventionStartEndMetadata.START: 'INT_START',
        InterventionStartEndMetadata.END: 'INT_END',
    },
    InterventionEvent.HYPOTHESIS_EVENT: {
        InterventionHypothesisMetadata.ABSENT: 'HYP_ABSENT',
        InterventionHypothesisMetadata.SUSPECTED: 'HYP_SUSPECTED',
        InterventionHypothesisMetadata.CONFIRMED: 'HYP_CONFIRMED',
    },
    InterventionEvent.ACTION_EVENT: 'INT_ACTION',
}

def get_event_type(event):
    global EVENT_TYPE_DICT
    event_type = None
    if event.type == InterventionEvent.START_OR_END_EVENT:
        event_type = EVENT_TYPE_DICT[event.type][event.start_end_metadata.status]
    elif event.type == InterventionEvent.HYPOTHESIS_EVENT:
        event_type = EVENT_TYPE_DICT[event.type][event.hypothesis_metadata.status]
    elif event.type == InterventionEvent.ACTION_EVENT:
        event_type = EVENT_TYPE_DICT[event.type]
    return event_type


RESUME_HINT_DICT = {
    getattr(RequestAssistanceResult, x): x.lower()
    for x in dir(RequestAssistanceResult)
    if x.isupper()
}


# The tracer class

class Tracer(object):
    """
    Monitors the actions taken during an intervention and logs them to enable
    continual learning of recovery actions. This class functions similarly to
    the Tracer class, which monitors the robot execution trace.
    """

    INTERVENTION_TRACE_TOPIC = '/intervention_monitor/trace'
    MAX_TRACE_LENGTH = 999

    # Stub event type definition
    TIME_EVENT = ExecutionTracer.TIME_EVENT

    # The expected events to appear on this stream. Most are autopopulated if
    # AUTO_INCLUDE_* flags are turned on
    EXCLUDE_HYPOTHESIS_EVENTS = set([])
    INCLUDE_HYPOTHESIS_EVENTS = []

    EXCLUDE_ACTION_EVENTS = set([])
    INCLUDE_ACTION_EVENTS = []

    EXCLUDE_START_EVENTS = set([])
    INCLUDE_START_EVENTS = []

    EXCLUDE_END_EVENTS = set([])
    INCLUDE_END_EVENTS = []

    # Flags to autopopulate the types of events to include or exclude
    AUTO_INCLUDE_HYPOTHESIS_EVENTS = True   # Populated from Annotations
    AUTO_INCLUDE_ACTION_EVENTS = True       # Populated from InterventionActionMetadata
    AUTO_INCLUDE_START_EVENTS = True        # Populated from ExecutionTracer.INCLUDE_TASK_STEP_EVENTS
    AUTO_INCLUDE_END_EVENTS = True          # Populated from RequestAssistanceResult

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
            Tracer.INTERVENTION_TRACE_TOPIC,
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
            if cls.AUTO_INCLUDE_ACTION_EVENTS:
                cls.INCLUDE_ACTION_EVENTS += [
                    getattr(InterventionActionMetadata, x) for x in sorted(dir(InterventionActionMetadata))
                    if (
                        x.isupper()
                        and getattr(InterventionActionMetadata, x) not in cls.EXCLUDE_ACTION_EVENTS
                        and getattr(InterventionActionMetadata, x) not in cls.INCLUDE_ACTION_EVENTS
                    )
                ]

            # Auto generate the start events if the flag is set
            if cls.AUTO_INCLUDE_START_EVENTS:
                # Initialize the ExecutionTracer trace types
                _ = ExecutionTracer.trace_types

                cls.INCLUDE_START_EVENTS += [
                    x for x in ExecutionTracer.INCLUDE_TASK_STEP_EVENTS
                    if (
                        x not in cls.EXCLUDE_START_EVENTS
                        and x not in cls.INCLUDE_START_EVENTS
                    )
                ]

            # Auto generate the end events if the flag is set
            if cls.AUTO_INCLUDE_END_EVENTS:
                cls.INCLUDE_END_EVENTS += [
                    getattr(RequestAssistanceResult, x) for x in sorted(dir(RequestAssistanceResult))
                    if (
                        x.isupper()
                        and getattr(RequestAssistanceResult, x) not in cls.EXCLUDE_END_EVENTS
                        and getattr(RequestAssistanceResult, x) not in cls.INCLUDE_END_EVENTS
                    )
                ]

            cls._trace_types = (
                [(Tracer.TIME_EVENT, None, 'time')]
                + [(InterventionEvent.HYPOTHESIS_EVENT, getattr(InterventionHypothesisMetadata, y), x)
                   for y in dir(InterventionHypothesisMetadata) if y.isupper() and y != 'ABSENT'
                   for x in cls.INCLUDE_HYPOTHESIS_EVENTS]
                + [(InterventionEvent.START_OR_END_EVENT, InterventionStartEndMetadata.START, x)
                   for x in cls.INCLUDE_START_EVENTS]
                + [(InterventionEvent.START_OR_END_EVENT, InterventionStartEndMetadata.END, x)
                    for x in cls.INCLUDE_END_EVENTS]
                + [(InterventionEvent.ACTION_EVENT, None, x) for x in cls.INCLUDE_ACTION_EVENTS]
            )

        return cls._trace_types

    @classproperty
    def trace_types_idx(cls):
        if cls._trace_types_idx is None:
            cls._trace_types_idx = { x: i for i, x in enumerate(Tracer.trace_types) }
        return cls._trace_types_idx

    @property
    def num_interventions(self):
        return len(self.full_traces)

    @property
    def last_event(self):
        if self.num_interventions == 0:
            return None
        return self.full_traces[-1][-1]

    def num_events(self, trace_idx):
        return len(self.full_traces[trace_idx])

    def trace(self, trace_idx):
        return self._traces[trace_idx][:, :self.num_events(trace_idx)]

    def start(self):
        self._should_trace = True

    def stop(self):
        self._should_trace = False

    def initialize_traces(self):
        self.full_traces.append(collections.deque(maxlen=Tracer.MAX_TRACE_LENGTH))
        if self._create_parsed_events:
            self.parsed_traces.append(collections.deque(maxlen=Tracer.MAX_TRACE_LENGTH))
        self._traces.append(np.ones((len(Tracer.trace_types), Tracer.MAX_TRACE_LENGTH,), dtype=np.float) * np.nan)

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
            and (msg.type, msg.start_end_metadata.status, msg.start_end_metadata.request.component,)
                not in Tracer.trace_types_idx
        ):
            return True


        if (
            msg.type == InterventionEvent.START_OR_END_EVENT
            and msg.start_end_metadata.status == InterventionStartEndMetadata.END
            and (msg.type, msg.start_end_metadata.status, msg.start_end_metadata.response.resume_hint)
                not in Tracer.trace_types_idx
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
            and (msg.type, InterventionHypothesisMetadata.CONFIRMED, msg.hypothesis_metadata.name)
                not in Tracer.trace_types_idx
        ):
            return True

        if (
            msg.type == InterventionEvent.ACTION_EVENT
            and (msg.type, None, msg.action_metadata.type) not in Tracer.trace_types_idx
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
            rospy.logwarn("Attempting to add to non-existent intervention")
            return True

        if (
            self.last_event is not None
            and (self.last_event.type != InterventionEvent.START_OR_END_EVENT
                or self.last_event.start_end_metadata.status != InterventionStartEndMetadata.END)
        ) and (
            msg.type == InterventionEvent.START_OR_END_EVENT
            and msg.start_end_metadata.status == InterventionStartEndMetadata.START
        ):
            rospy.logwarn("Attempting to start in the middle of another intervention")
            return True

        # All is well, include in the trace
        return False

    def update_trace(self, msg):
        if self.exclude_from_trace(msg):
            rospy.logwarn("Discarding event @ {} of type ({})".format(msg.stamp, msg.type))
            return

        # If this is an unknown task failure, and the execution tracer is set to
        # monitor unknown tasks, then update the message
        if msg.type == InterventionEvent.START_OR_END_EVENT \
                and msg.start_end_metadata.status == InterventionStartEndMetadata.START \
                and msg.start_end_metadata.request.component not in Tracer.INCLUDE_START_EVENTS \
                and ExecutionTracer.INCLUDE_UNKNOWN_TASK_EVENTS:
            msg.start_end_metadata.request.component = ExecutionTracer.UNKNOWN_TASK_NAME

        # Append the full trace
        if msg.type == InterventionEvent.START_OR_END_EVENT \
                and msg.start_end_metadata.status == InterventionStartEndMetadata.START:
            self.initialize_traces()

        num_events = self.num_events(-1)
        self.full_traces[-1].append(msg)
        if self._create_parsed_events:
            self.parsed_traces[-1].append(self._get_parsed_event_from_event(msg))

        # If the trace is too long, throw an error and stop updating the arrays
        if num_events > Tracer.MAX_TRACE_LENGTH:
            rospy.logwarn("Trace Length of {} > Max Length {}. Not tracking".format(
                num_events, Tracer.MAX_TRACE_LENGTH
            ))
            return

        # Copy the previous time stamp over
        self._traces[-1][:, num_events] = self._traces[-1][:, num_events-1]
        current_event = self._traces[-1][:, num_events]
        current_event[0] = msg.stamp.to_time()

        # Based on the type, update the trace
        if msg.type == InterventionEvent.START_OR_END_EVENT \
                and msg.start_end_metadata.status == InterventionStartEndMetadata.START:
            row = Tracer.trace_types_idx[(msg.type,
                                          msg.start_end_metadata.status,
                                          msg.start_end_metadata.request.component,)]
            current_event[row] = 1.0
        elif msg.type == InterventionEvent.START_OR_END_EVENT \
                and msg.start_end_metadata.status == InterventionStartEndMetadata.END:
            row = Tracer.trace_types_idx[(msg.type,
                                          msg.start_end_metadata.status,
                                          msg.start_end_metadata.response.resume_hint,)]
            current_event[row] = 1.0
        elif msg.type == InterventionEvent.ACTION_EVENT:
            row = Tracer.trace_types_idx[(msg.type, None, msg.action_metadata.type,)]
            current_event[row] = 1.0
        elif msg.type == InterventionEvent.HYPOTHESIS_EVENT:
            rows = [
                Tracer.trace_types_idx[(msg.type,
                                        InterventionHypothesisMetadata.SUSPECTED,
                                        msg.hypothesis_metadata.name,)],
                Tracer.trace_types_idx[(msg.type,
                                        InterventionHypothesisMetadata.CONFIRMED,
                                        msg.hypothesis_metadata.name,)]
            ]
            if msg.hypothesis_metadata.status == InterventionHypothesisMetadata.ABSENT:
                current_event[rows] = np.array([np.nan, np.nan])
            elif msg.hypothesis_metadata.status == InterventionHypothesisMetadata.SUSPECTED:
                current_event[rows] = np.array([1.0, np.nan])
            elif msg.hypothesis_metadata.status == InterventionHypothesisMetadata.CONFIRMED:
                current_event[rows] = np.array([np.nan, 1.0])

    def _get_parsed_event_from_event(self, event):
        global RESUME_HINT_DICT

        # Parsed events for interventions have no 'value'
        parsed_event = { 'time': event.stamp.to_time(),
                         'type': get_event_type(event),
                         'value': None, }

        if event.type == InterventionEvent.START_OR_END_EVENT:
            parsed_event['name'] = (
                event.start_end_metadata.request.component
                if event.start_end_metadata.status == InterventionStartEndMetadata.START
                else RESUME_HINT_DICT[event.start_end_metadata.response.resume_hint]
            )
        elif event.type == InterventionEvent.HYPOTHESIS_EVENT:
            parsed_event['name'] = event.hypothesis_metadata.name
        elif event.type == InterventionEvent.ACTION_EVENT:
            parsed_event['name'] = event.action_metadata.type
        else:
            parsed_event['type'] = None
            parsed_event['name'] = None

        return parsed_event
