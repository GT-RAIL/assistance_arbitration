#!/usr/bin/env python
# The main action server that provides the remote recovery behaviours

from __future__ import print_function, division

import os
import re
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

from assistance_arbitrator.intervention_tracer import Tracer as InterventionTracer


# The server performs local behaviours ro resume execution after contacting a
# remote human

class RemoteRecoveryServer(object):
    """
    Given a request for assistance, this class interfaces with the robot's
    remote monitoring UIs to deal with the error
    """

    # The services to enable and disable the controller
    ENABLE_SERVICE = '/remote_controller/enable'
    DISABLE_SERVICE = '/remote_controller/disable'

    # The service to call when the remote controller needs to indicate that the
    # recovery is complete and that it should be polled for the resume strategy.
    # Ideally, this would be specified somewhere other than here
    INTERVENTION_COMPLETE_SERVICE = '/remote_strategy/intervention_complete'

    def __init__(self):
        # The service proxies to enable and disable the controller
        self._controller_enable_srv = rospy.ServiceProxy(RemoteRecoveryServer.ENABLE_SERVICE, EnableRemoteControl)
        self._controller_disable_srv = rospy.ServiceProxy(RemoteRecoveryServer.DISABLE_SERVICE, DisableRemoteControl)

        # The intervention trace publisher
        self._trace_pub = rospy.Publisher(
            InterventionTracer.INTERVENTION_TRACE_TOPIC,
            InterventionEvent,
            queue_size=10
        )

        # The rviz interface and path
        try:
            self._rviz_view = RemoteRecoveryServer._parse_find_tag_in_str(
                rospy.get_param("~rviz", "$(find remote_strategy)/rviz/default.rviz")
            )
        except Exception as e:
            rospy.logerr("Remote: Unable to parse RViz view - {}".format(e))
            self._rviz_view = None
        self._rviz_process = None

        # A service that can be called when the recovery process is complete
        self._completion_service = rospy.Service(
            RemoteRecoveryServer.INTERVENTION_COMPLETE_SERVICE,
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
            ["rosrun", "rviz", "rviz", "-d", self._rviz_view]
        )
        self._rviz_process.wait()
        self._rviz_process = None

        # Get the desired resumption strategy
        disable_resp = self._controller_disable_srv()
        result.resume_hint = disable_resp.response.resume_hint
        result.context = disable_resp.response.context
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

    @staticmethod
    def _parse_find_tag_in_str(string):
        # If the value is set to false/null, then do not start the RViz process
        if not string:
            return None

        # Else, parse out the view to use
        path_components = string.split('/')
        match = re.match(r'^\$\(find (?P<package_name>\w+)\)$', path_components[0])
        if match is not None:
            package_path = rospkg.RosPack().get_path(match.groupdict()['package_name'])
            path_components[0] = package_path
        path = os.path.join(*path_components)
        assert os.path.exists(path), "Parsed path '{}' does not exist".format(path)
        return path
