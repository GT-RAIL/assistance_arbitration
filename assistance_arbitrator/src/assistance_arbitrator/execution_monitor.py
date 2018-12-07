#!/usr/bin/env python
# This class collects information from the various monitors and provides
# fault identification and localization as necessary

from __future__ import print_function, division

import rospy

from assistance_arbitrator.grapher import Grapher
from assistance_arbitrator.tracer import Tracer


# The main monitor class

class ExecutionMonitor(object):
    """
    This class is the entrypoint into the combined information available for
    diagnoses
    """

    def __init__(self):
        # Supplementary execution monitors
        self.rosgraph_monitor = Grapher()
        self.trace_monitor = Tracer()

    def start(self):
        # Nothing needs to be done on a start
        pass
