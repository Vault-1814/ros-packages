import rospy
import math
import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError


def getCVImage(data):
    """ returns cvImage"""
    bridge = CvBridge()
    img = None
    shape = None
    try:
        img = bridge.imgmsg_to_cv2(data, "bgr8")
        img = cv2.flip(img, -1)
        shape = img.shape
        shape = (shape[0], shape[1], math.sqrt(shape[0] ** 2 + shape[1] ** 2))
    except CvBridgeError, e:
        rospy.loginfo("Conversion failed: %s", e.message)
    return img, shape


def getMsgImage(cv_image):
    """ return ros's msg.Image"""
    bridge = CvBridge()
    msg_image = None
    try:
        msg_image = bridge.cv2_to_imgmsg(cv_image, "bgr8")
    except CvBridgeError, e:
        rospy.loginfo("Conversion failed: %s", e.message)
    return msg_image


def clearCanvas(canvas):
    """ return white filled canvas"""
    canvas[:, :, :] = (255, 255, 255)
    return canvas


def getCanvas(h, w):
    """ return white canvas for specified height and width"""
    canvas = np.zeros((h, w, 3), dtype=np.uint8)
    canvas = clearCanvas(canvas)
    return canvas


def getDimImage(l, hfov=58, vfov=45, dfov=70):
    """
        returns width and height of an image in [mm]
    """
    hfov_rad = hfov * math.pi / 180
    vfov_rad = vfov * math.pi / 180
    dfov_rad = dfov * math.pi / 180
    width = 2 * l * math.tan(hfov_rad / 2)
    height = 2 * l * math.tan(vfov_rad / 2)
    diag = 2 * l * math.tan(dfov_rad / 2)
    return width, height, diag


def getRatio(imShape, imDim):
    """ returns ratio for each of the known camera' angles"""
    ratioHFOV = imDim[0] / imShape[0]
    ratioVFOV = imDim[1] / imShape[1]
    ratioDFOV = imDim[2] / imShape[2]
    ratio = (ratioHFOV, ratioVFOV, ratioDFOV)
    return ratio




