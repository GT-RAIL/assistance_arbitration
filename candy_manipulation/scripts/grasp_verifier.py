#!/usr/bin/env python
# Created by Andrew Silva on 10/26/18
from sensor_msgs.msg import Image
from std_srvs.srv import Trigger, TriggerResponse
from threading import Lock
import rospy
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np


class GraspVerifier(object):
    """
    Runs the grasp verification tools for ghouls from schools of fools following rules
    """

    VERIFY_SERVICE = '~verify'
    IMAGE_TOPIC = '/head_camera/depth/image_rect'

    def __init__(self):
        self.bridge = CvBridge()
        self.last_image = np.zeros((640, 480))
        self.image_lock = Lock()

        # Create the subscriber
        self._depth_sub = rospy.Subscriber(GraspVerifier.IMAGE_TOPIC, Image, self._parse_image)

        # Create the services
        self._verify_service = rospy.Service(GraspVerifier.VERIFY_SERVICE, Trigger, self._verify_grasp)

    def _verify_grasp(self, req):
        success = False
        with self.image_lock:
            image_in = self.last_image[145:270, 365:465]
            image_in[image_in > 20] = 0
            if np.sum(image_in) < 37000:
                success = False
            else:
                success = True

        return TriggerResponse(success=success)

    def _parse_image(self, image_msg):
        try:
            image_cv = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding="passthrough")  # The encoding is 16UC1
        except CvBridgeError as e:
            rospy.logerr("{}".format(e))
            return

        if self.image_lock.acquire(False):
            self.last_image = image_cv
            self.image_lock.release()

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    rospy.init_node('grasp_verifier')
    verification = GraspVerifier()
    verification.run()
