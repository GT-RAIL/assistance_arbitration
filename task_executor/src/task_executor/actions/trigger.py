#!/usr/bin/env python
# Trigger a service

from __future__ import print_function, division

import rospy

from task_executor.abstract_step import AbstractStep

from std_srvs.srv import Trigger


class TriggerAction(AbstractStep):
    """
    Trigger the service at :const:`TRIGGER_SERVICE_NAME`
    """

    TRIGGER_SERVICE_NAME = '/test_trigger'

    def init(self, name):
        self.name = name
        self._trigger_srv = rospy.ServiceProxy(TriggerAction.TRIGGER_SERVICE_NAME, Trigger)

        # Set a stop flag
        self._stopped = False

        # Wait for a connection to the trigger
        rospy.loginfo("Connecting to the trigger service...")
        self._trigger_srv.wait_for_service()
        rospy.loginfo("...trigger service connected")

    def run(self):
        """
        The run function for this step

        Yields:
            success (bool) : The returned value from the trigger

        .. seealso::

            :meth:`task_executor.abstract_step.AbstractStep.run`
        """
        rospy.loginfo("Action {}: Triggering service".format(self.name))
        self._stopped = False

        # Fetch the beliefs and create the return dictionary
        resp = self._trigger_srv()
        yield self.set_running()  # Check the status of the server

        # Return the appropriate dictionary
        if self._stopped:
            yield self.set_preempted(
                action=self.name,
                srv=Trigger.TRIGGER_SERVICE_NAME
            )
        else:
            yield self.set_succeeded(success=resp.success)

    def stop(self):
        self._stopped = True
