import rospy
import math
import numpy as np
from cv_bridge import CvBridge, CvBridgeError


def getCVImage(data):
    bridge = CvBridge()
    img = None
    shape = None
    try:
        img = bridge.imgmsg_to_cv2(data, "bgr8")
        shape = img.shape
        shape = (shape[0], shape[1], math.sqrt(shape[0] ** 2 + shape[1] ** 2))
    except CvBridgeError, e:
        rospy.loginfo("Conversion failed: %s", e.message)
    return img, shape


def getMsgImage(cv_image):
    bridge = CvBridge()
    msg_image = None
    try:
        msg_image = bridge.cv2_to_imgmsg(cv_image, "bgr8")
        # msg_image = bridge.cv2_to_imgmsg(cv_image, "8UC1")
    except CvBridgeError, e:
        rospy.loginfo("Conversion failed: %s", e.message)
    return msg_image


def clearCanvas(canvas):
    canvas[:, :, :] = (255, 255, 255)
    return canvas


def getCanvas(h, w):
    canvas = np.zeros((h, w, 3), dtype=np.uint8)
    canvas = clearCanvas(canvas)
    return canvas


def getDimImage(l, hfov=58, vfov=45, dfov=70):
    """
        hfov and vfov for asus xtion pro live
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
    ratioHFOV = imDim[0] / imShape[0]
    ratioVFOV = imDim[1] / imShape[1]
    ratioDFOV = imDim[2] / imShape[2]
    ratio = (ratioHFOV, ratioVFOV, ratioDFOV)
    return ratio




