import numpy as np
import cv2
import imutils
from scipy.spatial import distance as dist
from imutils import perspective
from imutils import contours
import base64
import time
import urllib2


def midpoint(ptA, ptB):
    return ((ptA[0] + ptB[0]) * 0.5, (ptA[1] + ptB[1]) * 0.5)

class VideoCV:

    def __init__(self, source):
        self.cam = source
        self.pixelsPerMetric = 1
        self.WIDTH_PIXEL = 85 # nexus 5 1920x1080
        self.em = 'px'

    def getRawFrame(self, resize=False, height=300, width=300):
        frame = self.cam.getFrame()
        if resize:
            frame = cv2.resize(frame, (height, width))
        return frame

    def getGrayBlure(self, raw):
        gray = cv2.cvtColor(raw, cv2.COLOR_BGR2GRAY)
        blured = cv2.GaussianBlur(gray, (5, 5), 0)
        return blured

    def getThresholdImage(self, img):
        img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        img_blure = cv2.GaussianBlur(img_gray, (5, 5), 0)
        #thresh = cv2.adaptiveThreshold(img_blure, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 11, 2)
        ret, thresh = cv2.threshold(img_blure, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
        return thresh

    def getCandyEdges(self, gray):
        edged = cv2.Canny(gray, 70, 120)
        edged = cv2.dilate(edged, None, iterations=3)
        edged = cv2.erode(edged, None, iterations=2)
        return edged

    def getContours(self, thresh):
        contours, hierarchy = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        #contours = contours[0] if imutils.is_cv2() else contours[1]
        #(contours, _) = imutils.contours.sort_contours(contours)
        return contours

    # only for convex hull
    def getApprox(self, contour):
        hull = cv2.convexHull(contour)
        epsilon = 0.001 * cv2.arcLength(hull, True)
        approx = cv2.approxPolyDP(hull, epsilon, True)
        return approx

    def printGeometryOfObject(self, contour, orig):
        # hmmm...
        box = cv2.minAreaRect(contour)
        box = cv2.cv.BoxPoints(box)
        box = np.array(box, dtype="int")
        box = perspective.order_points(box)
        cv2.drawContours(orig, [box.astype("int")], -1, (0, 255, 0), 2)
        (tl, tr, br, bl) = box
        # middles of sides
        (tltrX, tltrY) = midpoint(tl, tr)
        (blbrX, blbrY) = midpoint(bl, br)
        (tlblX, tlblY) = midpoint(tl, bl)
        (trbrX, trbrY) = midpoint(tr, br)
        # compute the Euclidean distance between the midpoints
        dA = dist.euclidean((tltrX, tltrY), (blbrX, blbrY))
        dB = dist.euclidean((tlblX, tlblY), (trbrX, trbrY))
        if self.pixelsPerMetric is None:
            self.pixelsPerMetric = dB / self.WIDTH_PIXEL
        # compute the size of the object
        dimA = dA / self.pixelsPerMetric
        dimB = dB / self.pixelsPerMetric
        # ------------------
        info = "Height:%d\tWidth:%d\n" % (dimA, dimB)
        print(info)
        # ------------------
        # draw corner points
#        for (x, y) in box:
#            cv2.circle(orig, (int(x), int(y)), 5, (0, 0, 255), -1)
        # draw the midpoints on the image
#        cv2.circle(orig, (int(tltrX), int(tltrY)), 5, (255, 0, 0), -1)
#        cv2.circle(orig, (int(blbrX), int(blbrY)), 5, (255, 0, 0), -1)
#        cv2.circle(orig, (int(tlblX), int(tlblY)), 5, (255, 0, 0), -1)
#        cv2.circle(orig, (int(trbrX), int(trbrY)), 5, (255, 0, 0), -1)
        # draw lines between the midpoints

        return orig

    def drawContourEx(self, canvas, contour, color=(0, 0, 255)):
        cv2.drawContours(canvas, contour, -1, color, 3)
        mts = cv2.moments(contour)
        h, w = canvas.shape[:2]
        if mts['m00'] != 0:
            cx = int(mts['m10'] / mts['m00'])
            cy = int(mts['m01'] / mts['m00'])
            cv2.line(canvas, (0, cy), (2 * h, cy), (0, 0, 0))
            cv2.line(canvas, (cx, 0), (cx, 2 * w), (0, 0, 0))

    def getCanvas(self, h, w):
        canvas = np.zeros((h, w, 3), dtype=np.uint8)
        canvas[:, :, :] = (255, 255, 255)
        return canvas

    def clearCanvas(self, canvas):
        canvas[:, :, :] = (255, 255, 255)
        return canvas

    def run(self):
        frame = self.getRawFrame()
        h, w = frame.shape[:2]
        canvas = self.getCanvas(h, w)
        while True:
            self.clearCanvas(canvas)
            frame = self.getRawFrame()

            cv2.imshow('frame0', frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        self.cam.release()
        cv2.destroyAllWindows()

    def measures_demo(self):
        frame = self.getRawFrame()
        h, w = frame.shape[:2]
        canvas = self.getCanvas(h, w)
        while True:
            self.clearCanvas(canvas)
            frame = self.getRawFrame()
            gray = self.getGrayBlure(frame)
            edged = self.getCandyEdges(gray)
            cnts = self.getContours(edged)
            #self.drawContourEx(canvas, cnts)
            for c in cnts:
                if cv2.contourArea(c) < 500:
                    continue
                #orig = self.printGeometryOfObject(c, frame)
            cv2.imshow('frame2', frame)
            #cv2.imshow('frame3', canvas)
            #cv2.imshow('frame4', gray)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        self.cam.release()
        cv2.destroyAllWindows()

def main():
    url = 'http://192.168.1.33:4747/mjpegfeed?1920x1080'
    camera = Camera(1)
    videoCV = VideoCV(camera)
    videoCV.run()
    #videoCV.measures_demo()
main()
