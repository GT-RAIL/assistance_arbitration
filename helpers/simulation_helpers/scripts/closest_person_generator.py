#!/usr/bin/env python
# Generate a fake person near the robot when a service is called. This is
# essentially a stub to rail_people_detector/closest_person_node

from __future__ import print_function, division

import sys
import numpy as np

import rospy

from std_msgs.msg import Header, ColorRGBA
from rail_people_detection_msgs.msg import Person, DetectionContext
from geometry_msgs.msg import PoseStamped, Vector3
from visualization_msgs.msg import Marker

from std_srvs.srv import Trigger, TriggerResponse
from simulation_helpers.srv import CreatePerson, CreatePersonResponse


class PersonGenerator(object):

    def __init__(self):
        # The frame that the person will be output in
        self.desired_pose_frame = rospy.get_param("~desired_pose_frame")

        # No-op. Included here to keep parity with the interface of
        # closest_person_node
        self.position_match_threshold = rospy.get_param("~position_match_threshold")

        # The rate at which to publish the person once generated
        self.publish_rate = rospy.get_param("~publish_rate")

        # Whether to output debug topics or not
        self.debug_enabled = rospy.get_param("~debug", False)

        # Valiables to keep state
        self._closest_person = None

        # Services and topics
        self._create_person_srv = rospy.Service("~create_person", CreatePerson, self._create_person)
        self._destroy_person_srv = rospy.Service("~destroy_person", Trigger, self._destroy_person)
        self._closest_person_pub = rospy.Publisher("~closest_person", Person, queue_size=1)

        # Debug
        if self.debug_enabled:
            self._debug_pub = rospy.Publisher("~debug/1", Marker, queue_size=1)
            self._debug_marker = None

    def spin(self):
        rate = rospy.Rate(self.publish_rate)
        while not rospy.is_shutdown():
            if self._closest_person is not None:
                self._closest_person.header.stamp = rospy.Time.now()
                self._closest_person_pub.publish(self._closest_person)

            if self.debug_enabled and self._debug_marker is not None:
                # Assume that the marker is not None when the person is not None
                self._debug_marker.header = self._closest_person.header
                self._debug_pub.publish(self._debug_marker)

            rate.sleep()

    def _create_person(self, req):
        pose = PoseStamped(
            header=Header(frame_id=self.desired_pose_frame, stamp=rospy.Time.now())
        )

        # The position of the person is normally distributed 2m ahead, 0m
        # to the side, and 1.7m high. Standard deviations of 1m, 2m, and 0.05m
        # respectively. Assume a rotation of 0
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = \
            (np.random.randn(3) * [1, 2, 0.05]) + [2.0, 0.0, 1.7]
        pose.pose.orientation.w = 1.0

        rospy.loginfo("Person created at: {}".format(
            [pose.pose.position.x, pose.pose.position.y, pose.pose.position.z]
        ))

        # Update the position of the person and set the person to being a face
        # detection. This might need to be updated
        self._closest_person = Person(
            header=pose.header,
            pose=pose.pose,
            id=str(np.random.randint(100)) if not req.id else req.id,
            detection_context=DetectionContext(pose_source=DetectionContext.POSE_FROM_FACE)
        )

        # If debug, create the debug marker
        if self.debug_enabled:
            self._debug_marker = Marker(
                header=pose.header,
                ns="person_debug",
                id=int(self._closest_person.id),
                type=Marker.CYLINDER,
                action=Marker.ADD,
                pose=pose.pose,
                scale=Vector3(0.15, 0.3, pose.pose.position.z),
                color=ColorRGBA(0.0, 1.0, 0.0, 1.0)
            )
            self._debug_marker.pose.position.z = pose.pose.position.z / 2

        return CreatePersonResponse(person=self._closest_person)

    def _destroy_person(self, req):
        # First destroy the debug marker
        if self.debug_enabled and self._debug_marker is not None:
            self._debug_marker.action = Marker.DELETE
            self._debug_pub.publish(self._debug_marker)
            self._debug_marker = None

        # Then destroy the person
        self._closest_person = None

        return TriggerResponse(success=True)


if __name__ == '__main__':
    rospy.init_node('closest_person_generator')
    generator = PersonGenerator()
    generator.spin()
