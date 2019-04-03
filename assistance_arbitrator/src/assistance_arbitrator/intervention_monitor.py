#!/usr/bin/env python
# This monitors the different changes to the robot during an intervention and
# provides a summary of the actions and results over the course of the
# intervention, as needed

from __future__ import print_function, division

import sys
import collections
import numpy as np

import rospy

from assistance_msgs.msg import (RequestAssistanceActionGoal,
                                 RequestAssistanceResult, InterventionEvent,
                                 InterventionHypothesisMetadata,
                                 InterventionActionMetadata)

from assistance_arbitrator.tracer import Tracer

# Import isolation
sys.path.append('/home/banerjs/Libraries/RAIL/codebase/banerjs/isolation/models')
from isolation.data.annotations import Annotations


# The main monitor class

class InterventionMonitor(object):
    """
    Monitors the actions taken during an intervention and logs them to enable
    continual learning of recovery actions. This class functions similarly to
    the Tracer class, which monitors the robot execution trace.
    """

    INTERVENTION_TRACE_TOPIC = '/intervention_monitor/trace'
    MAX_TRACE_LENGTH = 9999

    # Stub event type definition
    TIME_EVENT = Tracer.TIME_EVENT

    def __init__(self, start_time=None):
        start_time = start_time or rospy.Time.now()

        # Book-keeping variables to keep track of the intervention events

    def start(self):
        pass

    def stop(self):
        pass
