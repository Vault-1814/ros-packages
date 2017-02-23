#!/usr/bin/env python
import rospy
import cv2
import numpy as np
import math
from scipy.spatial import distance as dist
from imutils import perspective
from client import *
import libs.geometry as g
import libs.utils as u
from cvision.msg import Object
from cvision.msg import ListObjects
from cvision.msg import Orientation

MM_TO_M = 0.001


class Measuring:

    def __init__(self, imageInfo):
        global DISSIMILARITY_THRESHOLD
        DISSIMILARITY_THRESHOLD = 5 

        with np.load('blue10.npz') as X:
            self.c = [X[i] for i in X]
        rospy.loginfo('blue blk is load!')

        # image size
        x, y, _ = imageInfo['shape']
        self.xy0 = (x, y)
        # ratio [mm/px]
        self.ratioHFOV = imageInfo['ratio'][0]
        self.ratioVFOV = imageInfo['ratio'][1]
        self.ratioDFOV = imageInfo['ratio'][2]
        # center RF
        self.CRF = (y / 2, x / 2)
        self.imageRF = ((0, -self.CRF[1]), (self.CRF[0], 0))

    def getObject(self, contour, image=None, shape='undefined'):
        obj = Object()
        box = cv2.minAreaRect(contour)
        box = cv2.cv.BoxPoints(box)
        box = np.array(box, dtype="int")
        box = perspective.order_points(box)
        if image is not None:
            cv2.drawContours(image, [box.astype("int")], -1, (0, 255, 0), 1)
        # coordinates of corners
        (tl, tr, br, bl) = box
        "object's position"
        objGRF = g.midpoint(tl, br)
        objCRF = (self.CRF[0] - objGRF[0], self.CRF[1] - objGRF[1])
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
        # for diagonal FOV
        dimD = dD * self.ratioDFOV
        alpha = math.atan2(dY, dX)
        dimX = dimD * math.cos(alpha)
        dimY = dimD * math.sin(alpha)
        "object's orientation. [-90; 90] from X vector in image RF"
        if dX >= dY:
            objVectorX = (tltrX - blbrX, tltrY - blbrY)
            angle = -g.angleBetween(self.imageRF[0], objVectorX)
        else:
            objVectorY = (tlblX - trbrX, tlblY - trbrY)
            angle = -g.angleBetween(self.imageRF[0], objVectorY)
        if abs(angle) > math.pi / 2:
            angle += math.pi
        "!!!just fine form"
        dimPx = (dX, dY, 0)
        dimMm = (dimX, dimY, 0)
        dimM = (dimX * MM_TO_M, dimY * MM_TO_M, 0)
        objCRFinM = (objCRF[1] * MM_TO_M, objCRF[0] * MM_TO_M, 0)
        objOrientation = (0, angle, 0)
	"sent to ALEX server"
	sendObject(objOrientation, objCRFinM)
        "packing object"
        obj.shape = ''
        obj.dimensions = dimM
        obj.coordinates_center_frame = objCRFinM
        obj.orientation = objOrientation
        for (x, y) in box:
            # top left corner of a object
            cv2.circle(image, (int(tl[0]), int(tl[1])), 5, (255, 0, 0), 2)

            cv2.putText(image, "{0:.1f}".format(angle * 180 / math.pi),
                        (int(0 + 50), int(0 + 25)),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

            # BLUE point in a center frame
            cv2.circle(image, (int(self.CRF[0]), int(self.CRF[1])), 5, (0, 0, 255), 2)
            # center of object in center coordinate system
            # cv2.circle(image, (int(xcc), int(ycc)), 5, (0, 255, 0), 2)
            # red point of a center object
            cv2.circle(image, (int(objGRF[0]), int(objGRF[1])), 5, (255, 0, 0), 2)

            # cross in a center frame
            cv2.line(image, (0, self.CRF[1]), (self.xy0[1], self.CRF[1]), (0, 0, 255), 1)
            cv2.line(image, (self.CRF[0], 0), (self.CRF[0], self.xy0[0]), (0, 0, 255), 1)

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
        return obj

    def getListObjects(self, image):
        detail = True
        list_obj = []
        blurred = cv2.medianBlur(image, 3)
        gray = cv2.cvtColor(blurred, cv2.COLOR_BGR2GRAY)
        #gray = cv2.bitwise_not(gray)
        threshold_min = 100
        _, blurred = cv2.threshold(gray, threshold_min, 255, cv2.THRESH_BINARY)
        v = np.median(image)
        sigma = 0.33
        canny_low = int(max(0, (1 - sigma) * v))
        canny_high = int(min(255, (1 + sigma) * v))
        edged = cv2.Canny(blurred, canny_low, canny_high)
        edged = cv2.dilate(edged, None, iterations=3)
        edged = cv2.erode(edged, None, iterations=2)


        screw_contour = self.c[0]
        contours, hierarchy = cv2.findContours(edged.copy(), cv2.RETR_EXTERNAL,
                                                  cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(image, contours, -1, (0, 255, 0), 3)

        obj_screw = None
        draw_contours = []  # (ret, contour)
        for contour in contours:
            if cv2.contourArea(contour) < 600:
                continue
            ret = cv2.matchShapes(contour, screw_contour, 1, 0)
            # print(ret)
            if obj_screw is None or ret < obj_screw[0]:
                obj_screw = (ret, contour)
            draw_contours.append((ret, contour))

        print('!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!', len(contours), obj_screw[0])

        if obj_screw[0] < DISSIMILARITY_THRESHOLD:
            list_obj.append(self.getObject(obj_screw[1], image))

        if detail:

            for ret, contour in draw_contours:
                if cv2.contourArea(contour) < 600:
                    M = cv2.moments(contour)
                    text_pos = (int(M['m10'] / M['m00']),
                                int(M['m01'] / M['m00'])) if M['m00'] else (0, 0)

                    mask = np.zeros(gray.shape, np.uint8)
                    cv2.drawContours(mask, [contour], 0, 255, -1)
                    cv2.drawContours(image, [contour], 0, (0,255,0), -1)
                    color = cv2.mean(image, mask)

                    if contour is obj_screw[1]:
                        text = 'Screw ' + str(ret)
                        cv2.drawContours(image, [contour], -1, color, 2)
                        cv2.putText(image, text, text_pos,
                                    cv2.FONT_HERSHEY_SIMPLEX, fontScale=0.8,
                                    color=(0, 0, 0), thickness=4)
                        cv2.putText(image, text, text_pos,
                                    cv2.FONT_HERSHEY_SIMPLEX, fontScale=0.8,
                                    color=(224, 255, 224), thickness=2)
                    else:
                        text = '{:.3f}'.format(ret)
                        cv2.drawContours(image, [contour], -1, color, 2)
                        cv2.putText(image, text, text_pos,
                                    cv2.FONT_HERSHEY_SIMPLEX, fontScale=0.8,
                                    color=(0, 0, 0), thickness=4)
                        cv2.putText(image, text, text_pos,
                                    cv2.FONT_HERSHEY_SIMPLEX, fontScale=0.8,
                                    color=(255, 255, 255), thickness=2)

        if detail:
            return list_obj, image
        else:
            return list_obj


    """
    def getListObjects(self, contours, details=False):
        listObjects = []
        for contour in contours:
            obj = self.getObject(contour)
            listObjects.append(obj)
        return listObjects
    """
