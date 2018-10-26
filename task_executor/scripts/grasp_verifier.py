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
    STOP_SERVICE = '~stop'
    SETUP_SERVICE = '~setup'
    TRIGGER_SERVICE = '~trigger'

    def __init__(self):
        # The background spinner
        self._background_spinner = None
        self._spin = False
        self._trigger = False  # TODO: Use a wake-word or something smarter
        self.bridge = CvBridge()
        self.image_sub_topic_name = rospy.get_param('~image_sub_topic_name', default='/head_camera/depth/image_rect')
        self.last_image = np.zeros((640, 480))
        self.image_lock = Lock()
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
            image_cv = self.bridge.imgmsg_to_cv2(image_msg, "uint16")
        except CvBridgeError as e:
            print e
            return -1
        self.last_image = image_cv

    def run(self):
        rospy.Subscriber(self.image_sub_topic_name, Image, self._parse_image)
        rospy.spin()


if __name__ == '__main__':
    rospy.init_node('grasp_verifier')
    verification = GraspVerifier()
    verification.run()
