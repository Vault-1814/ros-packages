import rospy
import cv_bridge
import cv2
import math


def getBorderType(bt):
    """for opencv 3.2"""
    bts = (
        cv2.BORDER_CONSTANT,
        cv2.BORDER_REPLICATE,
        cv2.BORDER_REFLECT,
        cv2.BORDER_WRAP,
        cv2.BORDER_REFLECT_101,
        cv2.BORDER_TRANSPARENT,
        cv2.BORDER_REFLECT101,
        cv2.BORDER_DEFAULT,
        cv2.BORDER_ISOLATED)
    bts_name = (
        'BORDER_CONSTANT',
        'BORDER_REPLICATE',
        'BORDER_REFLECT',
        'BORDER_WRAP',
        'BORDER_REFLECT_101',
        'BORDER_TRANSPARENT',
        'BORDER_REFLECT101',
        'BORDER_DEFAULT',
        'BORDER_ISOLATED')
    # print(bts_name[bt], ' was selected!')
    return bts[bt]


def getShape(sh):
    if sh == 0:
        return cv2.MORPH_RECT
    elif sh == 1:
        return cv2.MORPH_CROSS
    else:
        return cv2.MORPH_ELLIPSE

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

