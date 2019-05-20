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

    def __init__(self, start_time=None, create_parsed_events=False):
        start_time = start_time or rospy.Time.now()
        self._create_parsed_events = create_parsed_events

        # Book-keeping variables to keep track of the intervention events
        self.full_trace = collections.deque(maxlen=Tracer.MAX_TRACE_LENGTH)
        self.parsed_trace = collections.deque(maxlen=Tracer.MAX_TRACE_LENGTH)
        self._traces = []
        self._should_trace = False

        # Initialize the trace
        self.initialize_trace(start_time)

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
                + [
                    (InterventionEvent.HYPOTHESIS_EVENT, getattr(InterventionHypothesisMetadata, y), x)
                    for y in dir(InterventionHypothesisMetadata) if y.isupper() and y != 'ABSENT'
                    for x in cls.INCLUDE_HYPOTHESIS_EVENTS
                ]
                + [
                    (InterventionEvent.START_OR_END_EVENT, InterventionStartEndMetadata.START, x)
                    for x in cls.INCLUDE_START_EVENTS
                ]
                + [
                    (InterventionEvent.START_OR_END_EVENT, InterventionStartEndMetadata.END, x)
                    for x in cls.INCLUDE_END_EVENTS
                ]
                + [(InterventionEvent.ACTION_EVENT, None, x) for x in cls.INCLUDE_ACTION_EVENTS]
            )

        return cls._trace_types

    @classproperty
    def trace_types_idx(cls):
        if cls._trace_types_idx is None:
            cls._trace_types_idx = { x: i for i, x in enumerate(Tracer.trace_types) }
        return cls._trace_types_idx

    @property
    def num_events(self):
        return len(self.full_trace)

    @property
    def last_event(self):
        return self.full_trace[-1]

    @property
    def trace(self):
        # return self._trace[:, :self.num_events]
        pass

    def start(self):
        self._should_trace = True

    def stop(self):
        self._should_trace = False

    def initialize_trace(self, start_time):
        """Initialize the first trace event"""
        event = InterventionEvent(stamp=start_time)
        self.full_trace.append(event)
        if self._create_parsed_events:
            self.parsed_trace.append(self._get_parsed_event_from_event(event))

    def exclude_from_trace(self, msg):
        """Check to see if the message should be excluded from the trace"""
        if not self._should_trace:
            return True

        # Warn if this is an event type that is not recognized
        if msg.type not in [InterventionEvent.START_OR_END_EVENT,
                            InterventionEvent.HYPOTHESIS_EVENT,
                            InterventionEvent.ACTION_EVENT]:
            rospy.logwarn("Unknown event @ {} of type ({})".format(msg.stamp, msg.type))
            return True

        # All is well, include in the trace
        return False

    def update_trace(self, msg):
        if self.exclude_from_trace(msg):
            return

        # Append the full trace
        self.full_trace.append(msg)
        if self._create_parsed_events:
            self.parsed_trace.append(self._get_parsed_event_from_event(msg))

        # Then, perform more processing for the specific parts of the trace that
        # we are interested in

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
