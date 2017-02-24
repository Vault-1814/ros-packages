#!/usr/bin/env python
import roslib
from scipy.spatial import distance as dist
from imutils import perspective
import scripts.libs.geometry as g
import math
import scripts.libs.utils as u
from cvision.msg import Object
import rospy
import numpy as np
from roslib.rospack import rospack_depends

import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

pubSee = rospy.Publisher('seeing', Image, queue_size=1)
bridge = CvBridge()


def getObject(contour, image=None, shape='undefined'):
    box = cv2.minAreaRect(contour)
    box = cv2.cv.BoxPoints(box)
    box = np.array(box, dtype="int")
    box = perspective.order_points(box)
    if image is not None:
        cv2.drawContours(image, [box.astype("int")], -1, (0, 255, 0), 1)
    # coordinates of corners
    (tl, tr, br, bl) = box
    "object's dimensions [mm]"
    # middles of sides
    (tltrX, tltrY) = g.midpoint(tl, tr)
    (blbrX, blbrY) = g.midpoint(bl, br)
    (tlblX, tlblY) = g.midpoint(tl, bl)
    (trbrX, trbrY) = g.midpoint(tr, br)
    # compute the Euclidean distance between the midpoints of sides
    dX = dist.euclidean((tltrX, tltrY), (blbrX, blbrY))
    dY = dist.euclidean((tlblX, tlblY), (trbrX, trbrY))
    dD = math.sqrt(dX ** 2 + dY ** 2)
    a, b, _ = image.shape
    dpx = math.sqrt(a**2 + b**2)
    xpx = dist.euclidean(tl, br)
    x = 50
    l = 200
    dmm = dpx*x/xpx
    dalpha = math.atan2(dmm/2, l)
    print(dmm)
    print(dalpha*180/math.pi)
    for (x, y) in box:
        # cross in a center object
        # dA
        cv2.line(image, (int(tltrX), int(tltrY)), (int(blbrX), int(blbrY)),
                 (255, 0, 255), 1)
        # dB
        cv2.line(image, (int(tlblX), int(tlblY)), (int(trbrX), int(trbrY)),
                 (255, 0, 255), 1)

        cv2.putText(image, 'A',
                    (int(tltrX), int(tltrY)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        cv2.putText(image, 'B',
                    (int(tlblX), int(tlblY)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

def drawGrid(image, centerCross=True, r=True, frame=True, frame34=True):
    # 480x640
    h, w, _ = image.shape
    if r:
        resolutions = [
                          [640, 360],
                          [1280, 720],
                          [1920, 1080]
                      ][0:3]

        for r in resolutions:
            (x, y) = r
            pt1 = ((w - x) / 2, (h - y) / 2)
            pt2 = (pt1[0] + x, pt1[1] + y)
            cv2.rectangle(image,pt1, pt2, (255,0,255), 1)
            cv2.putText(image, str(r), (pt1[0]+5, pt1[1]+25), cv2.FONT_HERSHEY_COMPLEX, 0.7, (0,200,255), 1)
    if frame:
        # frame 10px
        cv2.rectangle(image, (10, 10), (w-10,h-10), (200, 0, 255))
    if centerCross:
        # cross center
        cv2.line(image, (w/2, 0), (w/2, h), (0,0,255))
        cv2.line(image, (0, h/2), (w, h/2), (0, 0, 255))
    if frame34:
        # # 3/4
        cv2.line(image, (w/3, 0), (w/3, h), (0, 200, 255))
        cv2.line(image, (w/3+w/3, 0), (w/3+w/3, h), (0, 200, 255))
        cv2.line(image, (0, h/3), (w, h/3), (0, 200, 255))
        cv2.line(image, (0, h/3+h/3), (w, h/3+h/3), (0, 200, 255))
    return image

def findSquare(image):
    list_obj = []
    image = cv2.medianBlur(image, 7)
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    # gray = cv2.bitwise_not(gray)
    # ret, th = cv2.threshold(gray, 100, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
    image = cv2.blur(image, (5, 5))
    v = np.median(image)
    sigma = 0.33
    canny_low = int(max(0, (1 - sigma) * v))
    canny_high = int(min(255, (1 + sigma) * v))
    edged = cv2.Canny(image, canny_low, canny_high)
    edged = cv2.dilate(edged, None, iterations=3)
    edged = cv2.erode(edged, None, iterations=2)

    contours, hierarchy = cv2.findContours(edged.copy(), cv2.RETR_EXTERNAL,
                                           cv2.CHAIN_APPROX_SIMPLE)
    with np.load('squar50.npz') as X:
        c = [X[i] for i in X]
    obj = None
    draw_contour = ''
    for contour in contours:
        if cv2.contourArea(contour) < 600:
            continue
        ret = cv2.matchShapes(contour, c[0], 1, 0)
        # print(ret)
        if obj is None or ret < obj[0]:
            obj = (ret, contour)
            draw_contour = contour


    print('!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!', len(contours), obj[0])
    cv2.drawContours(image, obj[1], -1, (0, 0, 255), 2)
    if obj[0] < 1:
        list_obj.append(getObject(obj[1], image))


def callback(data):
    image = bridge.imgmsg_to_cv2(data, "bgr8")
    image = drawGrid(image, frame34=False)
    findSquare(image)


    msg = bridge.cv2_to_imgmsg(image, "bgr8")
    pubSee.publish(msg)


def run():
    rospy.init_node('cv_recognizer', anonymous=False)
    subCamera = rospy.Subscriber('/usb_cam/image_rect', Image,
                                      callback)

    rospy.spin()


if __name__ == '__main__':
    try:
        run()
    except rospy.ROSInterruptException:
        pass
