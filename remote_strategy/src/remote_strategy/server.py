#!/usr/bin/env python
# The main action server that provides the remote recovery behaviours

from __future__ import print_function, division

import os
import pickle
import subprocess

import rospy
import rospkg
import actionlib

from actionlib_msgs.msg import GoalStatus
from assistance_msgs.msg import (RequestAssistanceAction,
                                 RequestAssistanceResult, InterventionEvent,
                                 InterventionStartEndMetadata)
from assistance_msgs.srv import (EnableRemoteControl, EnableRemoteControlRequest,
                                 DisableRemoteControl)
from std_srvs.srv import Trigger, TriggerResponse

from .controller import RemoteController


# The server performs local behaviours ro resume execution after contacting a
# remote human

class RemoteRecoveryServer(object):
    """
    Given a request for assistance, this class interfaces with the robot's
    remote monitoring UIs to deal with the error
    """

    DEFAULT_RVIZ_VIEW = os.path.join(
        rospkg.RosPack().get_path('remote_strategy'),
        "rviz/fetch.rviz"
    )

    def __init__(self):
        # The service proxies to enable and disable the controller
        self._controller_enable_srv = rospy.ServiceProxy(RemoteController.ENABLE_SERVICE, EnableRemoteControl)
        self._controller_disable_srv = rospy.ServiceProxy(RemoteController.DISABLE_SERVICE, DisableRemoteControl)

        # The intervention trace publisher
        self._trace_pub = rospy.Publisher(
            RemoteController.INTERVENTION_TRACE_TOPIC,
            InterventionEvent,
            queue_size=10
        )

        # The remote interface pointers
        self._rviz_process = None

        # A service that can be called when the recovery process is complete
        self._completion_service = rospy.Service(
            RemoteController.INTERVENTION_COMPLETE_SERVICE,
            Trigger,
            self._intervention_complete
        )

        # Instantiate the action server to perform the recovery
        self._server = actionlib.SimpleActionServer(
            rospy.get_name(),
            RequestAssistanceAction,
            self.execute,
            auto_start=False
        )

    def start(self):
        # Start the action server and indicate that we are ready
        self._server.start()
        rospy.loginfo("Remote strategy node ready...")

    def execute(self, goal):
        """Execute the request for assistance"""
        result = self._server.get_default_result()
        result.stats.request_received = rospy.Time.now()

        rospy.loginfo("Remote: Serving Assistance Request for: {} (status - {})"
                      .format(goal.component, goal.component_status))

        # Set the request as acked and update the intervention trace
        result.stats.request_acked = rospy.Time.now()
        trace_msg = InterventionEvent(stamp=result.stats.request_acked,
                                      type=InterventionEvent.START_OR_END_EVENT)
        trace_msg.start_end_metadata.status = InterventionStartEndMetadata.START
        trace_msg.start_end_metadata.request = goal
        self._trace_pub.publish(trace_msg)

        enable_req = EnableRemoteControlRequest(request=goal)
        self._controller_enable_srv(enable_req)

        # Start an rviz process and wait until it is shut
        self._rviz_process = subprocess.Popen(
            ["rosrun", "rviz", "rviz", "-d", RemoteRecoveryServer.DEFAULT_RVIZ_VIEW]
        )
        self._rviz_process.wait()
        self._rviz_process = None

        # Get the desired resumption strategy
        disable_resp = self._controller_disable_srv()
        result.resume_hint = disable_resp.response.resume_hint
        result.stats.request_complete = rospy.Time.now()
        trace_msg = InterventionEvent(stamp=result.stats.request_complete,
                                      type=InterventionEvent.START_OR_END_EVENT)
        trace_msg.start_end_metadata.status = InterventionStartEndMetadata.END
        trace_msg.start_end_metadata.response = result
        self._trace_pub.publish(trace_msg)

        # Then return
        self._server.set_succeeded(result)

    def stop(self):
        pass

    def _intervention_complete(self, req=None):
        if self._rviz_process is not None and self._rviz_process.poll() is None:
            self._rviz_process.terminate()
        return TriggerResponse(success=True)
