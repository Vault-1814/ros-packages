#!/usr/bin/env python
import rospy
import math
import os
import cv2
import imutils
import numpy as np
from scipy.spatial import distance as dist

from cvision.msg import Object

import libs.geometry as g

DISSIMILARITY_THRESHOLD = 0.01

MM_TO_M = 0.001
AREA_MIN = 3000
AREA_MAX = 35000
DESIRED_CONTOURE_NAMES = ['circle', 'balk_lego']
CONTOUR_FILES_EXT = '.npz'


class Measuring:
    """ class contains functions for processing image """

    def __init__(self, imageInfo, length):
        self.length = length

        # image size
        x, y, _ = imageInfo['shape']
        self.xy0 = (x, y)

        # ratio [mm/px]
        self.ratioHFOV = imageInfo['ratio'][0]
        self.ratioVFOV = imageInfo['ratio'][1]
        self.ratioDFOV = imageInfo['ratio'][2]  # 0.3058082527

        # center RF
        self.CRF = (y / 2, x / 2)
        self.imageRF = ((0, -self.CRF[1]), (self.CRF[0], 0))
        self.standardContours = []

        # reading predetermined contours
        dirPath = os.path.dirname(os.path.realpath(__file__))
        for fileName in DESIRED_CONTOURE_NAMES:
            fileName = dirPath + '/contours/' + fileName + CONTOUR_FILES_EXT
            with np.load(fileName) as X:
                cnt = [X[i] for i in X]
                self.standardContours.append(cnt)
        rospy.loginfo('CONTOURS WAS LOADED!')

    def orderPoints(self, pts):
        """ sort corners in CW order"""
        xSorted = pts[np.argsort(pts[:, 0]), :]
        leftMost = xSorted[:2, :]
        rightMost = xSorted[2:, :]
        leftMost = leftMost[np.argsort(leftMost[:, 1]), :]
        (tl, bl) = leftMost
        D = dist.cdist(tl[np.newaxis], rightMost, "euclidean")[0]
        (br, tr) = rightMost[np.argsort(D)[::-1], :]
        return np.array([tl, tr, br, bl], dtype="float32")

    def getObject(self, contour, image=None, shape='undefined'):
        """
            - measuring coordinates of contour;
            - measuring orientation;
            - measuring dimensions;
            - draw any info on frame
        """
        rospy.loginfo('OBJECT is ' + shape + '. Measuring...')
        obj = Object()
        box = cv2.minAreaRect(contour)
        box = cv2.cv.BoxPoints(box) if imutils.is_cv2() else cv2.boxPoints(box)
        box = np.array(box, dtype="int")
        # box = perspective.order_points(box)
        box = self.orderPoints(box)
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
        "object's orientation. [-90; 90] from the X vector in the image RF"
        if dX >= dY:
            objVectorX = (tltrX - blbrX, tltrY - blbrY)
            angle = -g.angleBetween(self.imageRF[0], objVectorX)
        else:
            objVectorY = (tlblX - trbrX, tlblY - trbrY)
            angle = -g.angleBetween(self.imageRF[0], objVectorY)
        if abs(angle) > math.pi / 2:
            # if angle must be positive but it is not!!
            if 0 < abs(angle) < math.pi / 4 and tl[0] < br[0]:
                angle += -math.pi    
            else:
                angle += math.pi
        "!!!just fine form"
        dimPx = (dX, dY, dD)
        dimMm = (dimX, dimY, 0)
        dimM = (dimX * MM_TO_M, dimY * MM_TO_M, 0)
        objCRFinM = (objCRF[1] * MM_TO_M * self.ratioDFOV,
                     objCRF[0] * MM_TO_M * self.ratioDFOV,
                     -self.length * MM_TO_M)  # holy cow!
        objOrientation = (0, angle, 0)

        rospy.loginfo('dims of object in [px]: ' + str(dimPx))

        "sent to ALEX server, but now it is not need, or noo, need, or no, not need again"
        # if self.flag:
        #     self.flag = False
        #     self.sendObject(objOrientation, objCRFinM)
        "packing object"
        obj.shape = shape
        obj.dimensions = dimM
        obj.coordinates_center_frame = objCRFinM
        obj.orientation = objOrientation
        for (x, y) in box:
            # top left corner of a object
            cv2.circle(image, (int(tl[0]), int(tl[1])), 5, (255, 0, 0), 2)

            cv2.putText(image, "{0:.1f}".format(angle * 180 / math.pi),
                        (int(0 + 50), int(0 + 25)),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

            # BLUE point of a center object
            cv2.circle(image, (int(objGRF[0]), int(objGRF[1])), 5, (255, 0, 0), 2)

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
        return obj, image

    def getListObjects(self, image):
        """
            - filter frame;
            - find contours;
            - filter contours;
            - detect shape;
            - collect it all together
        :param image: raw image form camera
        :return:
            - list of collect' objects
            - painted image
            - state = Thue, if objects was found
        """
        listObjects = []

        # filtering image
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        blur = cv2.medianBlur(gray, 3)
        v = np.median(blur)
        sigma = 0.33
        canny_low = int(max(0, (1 - sigma) * v))
        canny_high = int(min(255, (1 + sigma) * v))
        edged = cv2.Canny(blur, canny_low, canny_high)
        edged = cv2.dilate(edged, None, iterations=3)
        th = cv2.erode(edged, None, iterations=2)

        contours, hierarchy = cv2.findContours(th, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        # cv2.drawContours(image, contours, -1, (0, 200, 255))
        rospy.loginfo('total contours in the frame is ' + str(len(contours)))

        # remove all the smallest and the biggest contours
        # just both arrays clearing from trash
        wellContours = []
        wellHierarchy = []
        for i, contour in enumerate(contours):
            area = cv2.contourArea(contour)
            if AREA_MIN < area < AREA_MAX:
                wellContours.append(contour)
                wellHierarchy.append(hierarchy[0][i])

        # cv2.drawContours(image, wellContours, -1, (0, 0, 255))

        # compare and selecting desired contours
        foundContours = []
        for i, sCnt in enumerate(self.standardContours):   # wood, circle
            obj = None
            shape = DESIRED_CONTOURE_NAMES[i]
            for i, wCnt in enumerate(wellContours):
                # p = cv2.arcLength(wCnt, True)
                # wCnt = cv2.approxPolyDP(wCnt, 0.005 * p, True)
                h = wellHierarchy[i]
                if h[3] == -1:
                    ret = cv2.matchShapes(wCnt, sCnt[0], 1, 0)
                    if obj is None or ret < obj[0]:
                        obj = (ret, wCnt, shape)
                        foundContours.append(obj)

        test = []
        for f in foundContours:
            test.append(f[1])
        cv2.drawContours(image, test, -1, (0, 255, 255))

        rospy.loginfo('qty well contours: ' + str(len(wellContours)))
        rospy.loginfo('qty selected contours by matching: ' + str(len(foundContours)))

        # matching and measuring found contours
        if len(foundContours) != 0:
            for cnt in foundContours:
                if cnt[0] < DISSIMILARITY_THRESHOLD:
                    rospy.loginfo(cnt[2].upper() +
                                  ' | area: ' + str(cv2.contourArea(cnt[1])) +
                                  ' similr.: ' + str(cnt[0]))
                    o, image = self.getObject(cnt[1], image, shape=cnt[2])
                    listObjects.append(o)

        # RED point in a center frame
        cv2.circle(image, (int(self.CRF[0]), int(self.CRF[1])), 5, (0, 0, 255), 2)
        # cross in a center frame
        cv2.line(image, (0, self.CRF[1]), (self.xy0[1], self.CRF[1]), (0, 0, 255), 2)
        cv2.line(image, (self.CRF[0], 0), (self.CRF[0], self.xy0[0]), (0, 0, 255), 1)

        # scale for convenient
        scale = 0.5
        image = cv2.resize(image, (int(scale * self.xy0[1]), int(scale * self.xy0[0])))

        # TODO think about it :)
        state = False
        if len(listObjects) > 0:
            state = True
        return listObjects, image, state
