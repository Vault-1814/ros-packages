import rospy
import cv_bridge
import math


def getCVImage(data):
    bridge = cv_bridge.CvBridge()
    cv_image = None
    try:
        cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
    except cv_bridge.CvBridgeError, e:
        rospy.loginfo("Conversion failed: %s", e.message)
    return cv_image


def getMsgImage(cvImage):
    bridge = cv_bridge.CvBridge()
    imageMsg = None
    try:
        imageMsg = bridge.cv2_to_imgmsg(cvImage, 'bgr8')
    except cv_bridge.CvBridgeError, e:
        rospy.loginfo("Conversion failed: %s", e.message)
    return imageMsg

