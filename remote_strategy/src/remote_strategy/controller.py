#!/usr/bin/env python
# Provide an interface to the robot for remote assistance

from __future__ import print_function, division

import rospy

from assistance_msgs.msg import (RequestAssistanceResult, InterventionEvent,
                                 InterventionHypothesisMetadata,
                                 InterventionActionMetadata, BeliefKeys)
from assistance_msgs.srv import (EnableRemoteControl,
                                 EnableRemoteControlResponse,
                                 DisableRemoteControl,
                                 DisableRemoteControlResponse)
from std_srvs.srv import Trigger, TriggerResponse

from assistance_arbitrator.intervention_tracer import Tracer as InterventionTracer
from remote_strategy.server import RemoteRecoveryServer


# The app class that contains the app configuration and the controller

class RemoteController(object):
    """
    The default remote controller does nothing. It assumes that all
    interventions occur through RViz or the command line
    """

    def __init__(self):
        # The publisher of the trace
        self._trace_pub = rospy.Publisher(
            InterventionTracer.INTERVENTION_TRACE_TOPIC,
            InterventionEvent,
            queue_size=10
        )

        # Service proxy to indicate that the intervention is complete
        self._complete_intervention_srv = rospy.ServiceProxy(
            RemoteRecoveryServer.INTERVENTION_COMPLETE_SERVICE,
            Trigger
        )

        # Flags and services to enable and disable this controller
        self._current_error = None
        self._current_response = None
        self._enable_service = rospy.Service(
            RemoteRecoveryServer.ENABLE_SERVICE,
            EnableRemoteControl,
            self.enable
        )
        self._disable_service = rospy.Service(
            RemoteRecoveryServer.DISABLE_SERVICE,
            DisableRemoteControl,
            self.disable
        )

    def start(self):
        rospy.spin()

    def stop(self, *args, **kwargs):
        pass

    def enable(self, req):
        self._current_error = req.request
        if self._current_error.context:
            self._current_error.context = pickle.loads(self._current_error.context)
        self._current_response = None
        return EnableRemoteControlResponse()

    def disable(self, req=None):
        self._current_error = None
        # This happens if the RViz window is closed and there is no recovery
        # strategy that is provided. Default is to then exit from the task
        if self._current_response is None:
            self._current_response = RequestAssistanceResult(resume_hint=RequestAssistanceResult.RESUME_NONE)
        return DisableRemoteControlResponse(response=self._current_response)
