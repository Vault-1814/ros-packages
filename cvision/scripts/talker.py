#!/usr/bin/env python
import roslib
import rospy
from roslib.rospack import rospack_depends

import cv2
from cv_bridge import CvBridge, CvBridgeError
from cvision.msg import Object
from cvision.msg import ListObjects
from cvision.msg import Orientation
from sensor_msgs.msg import Image
from pycv import Recognize

CAMERA = 2
OPENCV_CAM = 0
ASUS_XTION_TOPIC = '/rgb/image'
USB_CAM_TOPIC = '/usb_cam/image_raw'
OPENCV_TOPIC = '/see_main_webcam'

def getTopicForCamera(camera):
    """
    :param camera: as device
    :param cv_cam: num camera for opencv stream (default is OFF)
    :return:
        if camera == 0: asus xtion
        if camera == 1: usb_cam topic
        if camera == 2: topic with opencv stream (default is OFF)
    """
    if camera == 0:
        return ASUS_XTION_TOPIC
    elif camera == 1:
        return USB_CAM_TOPIC
    elif camera == 2:
        if OPENCV_CAM == -1:
            rospy.logfatal("Opencv camera is off in 'talker.py' file!")
            raise rospy.ROSException
        return OPENCV_TOPIC


def talker():
    rospy.init_node('cv_recognizer', anonymous=False)
    bridge = CvBridge()
    cam = cv2.VideoCapture(0)
    pup_opencv_cam = rospy.Publisher('see_main_webcam', Image, queue_size=1)
    pub_orientation = rospy.Publisher('orientation', Orientation, queue_size=10)

    camera_topic = getTopicForCamera(CAMERA)
    # main callback there
    # TODO think about it
    Recognize(camera_topic, True)

    while not rospy.is_shutdown():
        # opencv camera read
        if OPENCV_CAM != -1:
            _, frame = cam.read()
            msg_cv = None
            try:
                # should create a special .msg file. may be. don't know...
                msg_cv = bridge.cv2_to_imgmsg(frame, "bgr8")
            except CvBridgeError, e:
                rospy.loginfo("Conversion failed: %s", e.message)
            pup_opencv_cam.publish(msg_cv)
        # height from manipulator
        msg = Orientation()
        msg.length = 110
        pub_orientation.publish(msg)
if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
