import cv2
import numpy as np
import math
from scipy.spatial import distance as dist
from imutils import perspective

import libs.geometry as g

from cvision.msg import Object
from cvision.msg import ListObjects
from cvision.msg import Orientation

MM_TO_M = 0.001


class Measuring:

    def __init__(self):

        goodContours = ''
        self.imshape_px = [0, 0, 0]
        self.imshape_mm = [0, 0, 0]

        x, y = 0, 0

        (x, y) = self.imshape_px[:2]
        self.CRF = (y / 2, x / 2)

        self.ratioHFOV = self.imshape_mm[0] / self.imshape_px[0]
        self.ratioVFOV = self.imshape_mm[1] / self.imshape_px[1]
        self.ratioDFOV = self.imshape_mm[2] / self.imshape_px[2]

        self.imageRF = ((0, -self.CRF[1]), (self.CRF[0], 0))

    def getObject(self, contour, shape='undefined'):
        obj = Object()
        box = cv2.minAreaRect(contour)
        box = cv2.cv.BoxPoints(box)
        box = np.array(box, dtype="int")
        box = perspective.order_points(box)
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
        objCRFinM = (objCRF[0] * MM_TO_M, objCRF[1] * MM_TO_M, 0)
        objOrientation = (0, angle, 0)
        "packing object"
        obj.shape = ''
        obj.dimensions = dimM
        obj.coordinates = objCRFinM
        obj.orientation = objOrientation
        return obj

    def getListObjects(self, contours, details=False):
        listObjects = []
        for contour in contours:
            obj = self.getObject(contour)
            listObjects.append(obj)
        return listObjects