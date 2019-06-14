#!/usr/bin/env python
# Get string data on a topic

from __future__ import print_function, division

import rospy

from task_executor.abstract_step import AbstractStep

from std_msgs.msg import String


# The action class

class GetStringAction(AbstractStep):
    """Get a string on a specified topic"""

    DEFAULT_STRING_TOPIC = "/test_string"

    def init(self, name):
        self.name = name
        self._stopped = False

    def run(self, topic=None, timeout=0.0):
        """
        The run function for this step

        Args:
            topic (str) : the name of the topic to get the string from; if not
                provided, use :const:`DEFAULT_STRING_TOPIC` as the default
            timeout (float) : the amount of time in which the string must be
                provided; if 0.0 (default), then wait forever

        Yields:
            string_data (str) : the string data in the indicated topic

        .. seealso::

            :meth:`task_executor.abstract_step.AbstractStep.run`
        """
        topic = topic or GetStringAction.DEFAULT_STRING_TOPIC
        rospy.loginfo("Action {}: Fetching data from {}".format(self.name, topic))
        self._stopped = False

        start_time = rospy.Time.now()
        end_time = start_time + rospy.Duration(timeout)
        string_data = None
        while (
            string_data is None
            or (timeout > 0 and rospy.Time.now() <= end_time)
        ):
            try:
                string_data = rospy.wait_for_message(topic, String, 0.1).data
            except rospy.ROSException as e:
                pass

            if self._stopped:
                break

            yield self.set_running()

        if self._stopped:
            yield self.set_preempted(
                action=self.name,
                topic=topic,
                timeout=timeout,
            )
        elif string_data is None:
            yield self.set_aborted(
                action=self.name,
                topic=topic,
                timeout=timeout,
            )
        else:  # Succeeded
            yield self.set_succeeded(string_data=string_data)

    def stop(self):
        self._stopped = True
