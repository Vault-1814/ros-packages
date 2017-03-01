#!/usr/bin/env python
import math

import cv2
import imutils
import numpy as np
from cvision.msg import Object
from scipy.spatial import distance as dist

import libs.geometry as g
import talker
from state_client import *

DISSIMILARITY_THRESHOLD = 0.1

MM_TO_M = 0.001
AREA_MIN = 1000
AREA_MAX = 100000
DESIRED_CONTOURE_NAMES = ['circle', 'wood']
CONTOUR_FILES_EXT = '.npz'


class Measuring:

    def __init__(self, imageInfo):
        print('MEASURING: ' + str(imageInfo))
        self.flag = True    # once run service. may remove!!
        self.foundObjects = [0, 0]  # bool -- found if wood and circle
        # objCnt = 'wood.npz' #'squar50.npz'
        #  objCnt = 'circle.npz'
        # with np.load(objCnt) as X:
        #     self.c = [X[i] for i in X]

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
        self.desiredContours = []
        print('MEASURING: ' + str(self.imageRF) + ' ' + str(self.CRF))
        for fileName in DESIRED_CONTOURE_NAMES:
            fileName += CONTOUR_FILES_EXT
            with np.load(fileName) as X:
                cnt = [X[i] for i in X]
                self.desiredContours.append(cnt)
        rospy.loginfo('CONTOURS WAS LOADED!')

    def orderPoints(self, pts):
        xSorted = pts[np.argsort(pts[:, 0]), :]
        leftMost = xSorted[:2, :]
        rightMost = xSorted[2:, :]
        leftMost = leftMost[np.argsort(leftMost[:, 1]), :]
        (tl, bl) = leftMost
        D = dist.cdist(tl[np.newaxis], rightMost, "euclidean")[0]
        (br, tr) = rightMost[np.argsort(D)[::-1], :]
        return np.array([tl, tr, br, bl], dtype="float32")

    def getObject(self, contour, image=None, shape='undefined'):
        rospy.loginfo('OBJECTS OK, getting...')
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
        print('dimImage: ' + str(self.xy0))
        print('DdimImage: ' + str(math.sqrt(self.xy0[0]**2 + self.xy0[1]**2)))
        print('ratio: ' + str(self.ratioDFOV))
        print('dimPx: ' + str(dimPx))
        print('')
        dimM = (dimX * MM_TO_M, dimY * MM_TO_M, 0)
        objCRFinM = (objCRF[1] * MM_TO_M * self.ratioDFOV,
                     objCRF[0] * MM_TO_M * self.ratioDFOV,
                     -talker.LENGTH * MM_TO_M)  # holy shit!
        objOrientation = (0, angle, 0)
        "sent to ALEX server, but now it is not need, and noo, need, and no, not need"
        # if self.flag:
        #     self.flag = False
        #     self.sendObject(objOrientation, objCRFinM)
        "packing object"
        obj.shape = shape
        obj.dimensions = dimM
        obj.coordinates_center_frame = objCRFinM
        obj.orientation = objOrientation
        # TODO
        if shape == 'wood':
            self.foundObjects[0] = 1
        elif shape == 'circle':
            self.foundObjects[1] = 1
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
        return obj, image

    def getListObjects(self, image):
        listObjects = []

        scale = 0.5
        image = cv2.resize(image, (int(scale * self.xy0[1]), int(scale * self.xy0[0])))

        # filtering image
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        blur = cv2.medianBlur(gray, 1)
        # blur = cv2.blur(gray, (5, 5))
        th = cv2.adaptiveThreshold(gray, 255, 1, cv2.THRESH_BINARY_INV, 11, 5)

        # v = np.median(image)
        # sigma = 0.33
        # canny_low = int(max(0, (1 - sigma) * v))
        # canny_high = int(min(255, (1 + sigma) * v))
        # edged = cv2.Canny(gray, 10, 20) #canny_low, canny_high)
        # edged = cv2.dilate(edged, None, iterations=1)
        # edged = cv2.erode(edged, None, iterations=1)

        contours, hierarchy = cv2.findContours(th.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(image, contours, -1, (255, 0, 255))
        print("total cnts: " + str(len(contours)))

        # remove all the smallest and the biggest contours
        wellContours = []
        for contour in contours:
            area = cv2.contourArea(contour)
            if AREA_MIN < area < AREA_MAX:
                wellContours.append(contour)
                print("area contour: " + str(area))

        # compare and selecting desired contours
        findedContours = []
        for i, dCnt in enumerate(self.desiredContours):   # wood, circle
            obj = None
            # print(i + len(self.desiredContours) - 1)
            shape = DESIRED_CONTOURE_NAMES[i]
            for wCnt in wellContours:
                # p = cv2.arcLength(wCnt, True)
                # wCnt = cv2.approxPolyDP(wCnt, 0.005 * p, True)
                ret = cv2.matchShapes(wCnt, dCnt[0], 1, 0)

                if obj is None or ret < obj[0]:
                    print(ret)
                    obj = (ret, wCnt, shape)
                    findedContours.append(obj)

        cv2.drawContours(image, wellContours, -1, (0, 0, 255))
        # cv2.drawContours(image, np.array(findedContours), -1, (255, 0, 0))

        print('*** qty contours: ' + str(len(wellContours)), 'satisfy contour: ' + str(len(findedContours)))

        # measuring found contours
        if len(findedContours) != 0:
            for cnt in findedContours:

                if cnt[0] < DISSIMILARITY_THRESHOLD:
                    print('OK area: ' + str(cv2.contourArea(cnt[1])),
                          ' similarity: ' + str(cnt[0]))
                    o, image = self.getObject(cnt[1], image, shape=cnt[2])
                    listObjects.append(o)

        # TODO
        state = False
        for fo in self.foundObjects:
            if fo == 1:
                state = True
        return listObjects, image, state
