#!/usr/bin/env python
# Sets the string at a specified topic

from __future__ import print_function, division

import rospy

from task_executor.abstract_step import AbstractStep

from std_msgs.msg import String


class SetStringAction(AbstractStep):
    """
    Set a string on a specified topic (latched). Cannot be preempted.
    """

    DEFAULT_STRING_TOPIC = "/test_string_out"

    def init(self, name):
        self.name = name

    def run(self, string_data, topic=None):
        """
        The run function for this step

        Args:
            string_data (str) : the string data to send on the topic
            topic (str) : the name of the topic to get the string from; if not
                provided, use :const:`DEFAULT_STRING_TOPIC` as the default

        .. seealso::

            :meth:`task_executor.abstract_step.AbstractStep.run`
        """
        topic = topic or SetStringAction.DEFAULT_STRING_TOPIC
        rospy.loginfo("Action {}: Sending data to {}".format(self.name, topic))

        # Publish the data
        publisher = rospy.Publisher(topic, String, queue_size=1, latch=True)
        publisher.publish(string_data)
        rospy.sleep(0.1)  # Let the publish actually happen

        # Set the result to succeeded
        yield self.set_succeeded()

    def stop(self):
        # This action cannot be stopped
        pass
