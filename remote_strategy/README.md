# Remote Strategy

A remote diagnosis and recovery strategy. It pulls up interfaces to specify the candidate diagnoses, an RViz view, and a means of specifying primitive actions.


## Notes

API for `remote_controller`:

```python
# Imports
from assistance_msgs.srv import (EnableRemoteControl,
                                 EnableRemoteControlResponse,
                                 DisableRemoteControl,
                                 DisableRemoteControlResponse)
from std_srvs.srv import Trigger, TriggerResponse

from remote_strategy.server import RemoteRecoveryServer

class RemoteController(object):

    def __init__(self):
        """
        Initialize the controller. It must create the service interfaces to the
        intervention enable and disable services; as shown below
        """

        # Service proxy to indicate that the intervention is complete
        self._complete_intervention_srv = rospy.ServiceProxy(
            RemoteRecoveryServer.INTERVENTION_COMPLETE_SERVICE,
            Trigger
        )

        # Service called by the server to enable the controller (essentially
        # notifying the user of the requirement for an intervention)
        self._enable_service = rospy.Service(
            RemoteRecoveryServer.ENABLE_SERVICE,
            EnableRemoteControl,
            self.enable
        )

        # Service called by the server to disable the controller. This happens
        # if the controller calls INTERVENTION_COMPLETE_SERVICE, or if the
        # Rviz window opened by the server is shutdown
        self._disable_service = rospy.Service(
            RemoteRecoveryServer.DISABLE_SERVICE,
            DisableRemoteControl,
            self.disable
        )

    def start(self):
        """Start the controller. The method should handle its own spinning"""
        rospy.spin()

    def stop(self, *args, **kwargs):
        """
        In case there needs to be a shutdown handler. User defined behaviour
        """
        pass

    def enable(self, req):
        """
        Request from the server to start allowing remote intervention on robot
        """
        # Do something...
        return EnableRemoteControlResponse()

    def disable(self, req):
        """
        Request from the server to ignore all subsequent interventions through
        the interface.
        """
        # Do something...
        return DisableRemoteControlResponse(...)
```
