import rospy
import cv2
import numpy as np
from scipy.spatial import distance as dist
from imutils import perspective
from sensor_msgs.msg import Image
from cvision.msg import Object
from cvision.msg import ListObjects


class Recognize:

    def __init__(self, source):
        self.pixelsPerMetric = 1
        self.WIDTH_PIXEL = 85 # nexus 5 1920x1080

        self.subscriber = rospy.Subscriber(source, Image, self.callback, queue_size=1)
        self.pubForManipulator = rospy.Publisher('list_objects', ListObjects)
        self.pubForPeople = rospy.Publisher('see_process_recognizing', ListObjects)

    def midpoint(self, ptA, ptB):
        return (ptA[0] + ptB[0]) * 0.5, (ptA[1] + ptB[1]) * 0.5

    def callback(self, data):
        #rospy.loginfo(data)
        raw = data.data
        gray = cv2.cvtColor(raw, cv2.COLOR_BGR2GRAY)
        edged = cv2.Canny(gray, 70, 120)
        edged = cv2.dilate(edged, None, iterations=3)
        edged = cv2.erode(edged, None, iterations=2)
        contours, hierarchy = cv2.findContours(edged, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        list = []
        for contour in contours:
            o = Object()
            if cv2.contourArea(c) < 500:
                continue
            box = cv2.minAreaRect(contour)
            box = cv2.cv.BoxPoints(box)
            box = np.array(box, dtype="int")
            box = perspective.order_points(box)
            # draw on source frame
            cv2.drawContours(raw, [box.astype("int")], -1, (0, 255, 0), 2)
            (tl, tr, br, bl) = box
            # middles of sides
            (tltrX, tltrY) = self.midpoint(tl, tr)
            (blbrX, blbrY) = self.midpoint(bl, br)
            (tlblX, tlblY) = self.midpoint(tl, bl)
            (trbrX, trbrY) = self.midpoint(tr, br)
            # compute the Euclidean distance between the midpoints
            dA = dist.euclidean((tltrX, tltrY), (blbrX, blbrY))
            dB = dist.euclidean((tlblX, tlblY), (trbrX, trbrY))
            if self.pixelsPerMetric is None:
                self.pixelsPerMetric = dB / self.WIDTH_PIXEL
            # compute the size of the object
            dimA = dA / self.pixelsPerMetric
            dimB = dB / self.pixelsPerMetric
            # ------------------
            o.shape = 'undefine'
            o.dimensions = (dimA, dimB, 1)
            # ------------------
            info = "Height:%d\tWidth:%d\n" % (dimA, dimB)
            #rospy.loginfo(info)
            list.append(o)
        msg = ListObjects()
        msg = list
        rospy.loginfo('Send %s objects', len(msg))
        self.pubForManipulator.publish(msg)
