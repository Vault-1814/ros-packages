import rospy
import cv2
import numpy as np
import math
import exmodules.utils as utils
from collections import deque
from cv_bridge import CvBridge, CvBridgeError
from scipy.spatial import distance as dist
from imutils import perspective
from sensor_msgs.msg import Image
from cvision.msg import Object
from cvision.msg import ListObjects
from cvision.msg import Position

TOPIC_OBJECTS = 'found_objects'
TOPIC_SEEING = 'seeing'
TOPIC_INTERIM_IMAGE = 'interim_image'

class FovVision:
    def __init__(self, imageRectTopic, cameraPositionTopic):
        self.imshapePX = [0, 0, 0]
        self.imshapeMM = [0, 0, 0]
        self.mmPerPx = 0

        """ for instance, ['gaussian_blur', 'canny']
            filters are added in forward order their applying to image
        """
        self.filtersChain = deque([], 5)    # may be stack ???
        self.filtersChain.append('gaussian_blur')
        self.filtersChain.append('canny')
        self.filtersChain.rotate()


        # subscribers
        self.subCamera = rospy.Subscriber(
            imageRectTopic, Image, self.cameraCallback)
        self.subPosition = rospy.Subscriber(
            cameraPositionTopic, Position, self.cameraPositionCallback, 1)

        # publisher
        self.pubListObjects = rospy.Publisher(
            TOPIC_OBJECTS, ListObjects, queue_size=1)
        self.pubSeeing = rospy.Publisher(
            TOPIC_SEEING, Image, queue_size=1)
        self.pubInterimImage = rospy.Publisher(
            TOPIC_INTERIM_IMAGE, Image, queue_size=1)

    def addFilterToChain(self, filterName):
        self.filtersChain.append(filterName)

    def addFiltersChain(self, filtersChain):
        self.filtersChain = filtersChain

    def lookAt(self, cvImage, detail=False):
        listObjects = []
        # make sure that the image is graaaaay
        if not utils.isGray(cvImage):
            cvImage = cv2.cvtColor(cvImage, cv2.COLOR_BGR2GRAY)
        util.applyFiltersChain(self.filtersChain)
        contours, hierarchy = cv2.findContours(
            wellPreparedImage, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        # TODO think about limitation for contours here
        for contour in contours:
            o = Object()
            if cv2.contourArea(contour) < 500:
                continue
            box = cv2.minAreaRect(contour)
            box = cv2.cv.BoxPoints(box)
            box = np.array(box, dtype="int")
            box = perspective.order_points(box)
            # coordinates of corners
            (tl, tr, br, bl) = box
            # middles of sides
            (tltrX, tltrY) = self.midpoint(tl, tr)
            (blbrX, blbrY) = self.midpoint(bl, br)
            (tlblX, tlblY) = self.midpoint(tl, bl)
            (trbrX, trbrY) = self.midpoint(tr, br)
            # compute the Euclidean distance between the midpoints
            dA = dist.euclidean((tltrX, tltrY), (blbrX, blbrY))
            dB = dist.euclidean((tlblX, tlblY), (trbrX, trbrY))
            # center of frame
            x0 = self.imshape_px[1] / 2
            y0 = self.imshape_px[0] / 2
            # center of object in center frame coord. system
            # coord. system is 5th joint of manipulator
            (xoc, yoc) = self.midpoint(tl, br)
            xcc = x0 - yoc
            ycc = y0 - xoc
            # rospy.loginfo("O(%s; %s) --> O'(%s; %s)", tltrX, tlblY, xcc, ycc)
            """
                Compute the size of the object.
                object_diag_mm = object_diag_px * frame_mm / frame_px
                !!! because angle for horizontal and vertical fields of view are different
                        and we do not know orientation object (still)
            """
            # [mm]
            ratio = self.imshape_mm[2] / self.imshape_px[2]
            dD = math.sqrt(dA ** 2 + dB ** 2)
            dimD = dD * ratio
            # rospy.loginfo("O_diag_mm = %s", dimD)
            alpha = math.atan2(dB, dA)
            dimA = dimD * math.sin(alpha)
            dimB = dimD * math.cos(alpha)
            s = "(%d,%d);(%d,%d);(%d,%d)==%f" % (
            x0, y0, xcc, ycc, yoc, xoc, ratio)
            rospy.loginfo(s)
            # ***
            # TODO detecting and analezing objects
            o.shape = 'undefined'
            o.dimensions = (dimA, dimB, 1)
            o.coordinates_center_frame = (ycc * ratio, xcc * ratio, 0)
            o.coordinates_frame = (tltrX * ratio, tlblY * ratio, 0)
            # info = "Height:%d\tWidth:%d\n" % (dimA, dimB)
            # rospy.loginfo(info)
            list.append(o)
            # ***
            if detail:
                # ------------------
                # draw on source frame
                cv2.drawContours(image, [box.astype("int")], -1, (0, 255, 0), 2)
                # draw corner points
                for (x, y) in box:
                    cv2.circle(image, (int(x), int(y)), 5, (0, 0, 255), -1)
                    cv2.line(image, (0, y0),
                             (self.imshape_px[1], y0), (0, 0, 255), 1)
                    cv2.line(image, (x0, 0),
                             (x0, self.imshape_px[0]), (0, 0, 255), 1)
                    cv2.line(image, (int(tltrX), int(tltrY)),
                             (int(blbrX), int(blbrY)), (255, 0, 255), 1)
                    cv2.line(image, (int(tlblX), int(tlblY)),
                             (int(trbrX), int(trbrY)), (255, 0, 255), 1)
                    """
                    # draw the midpoints on the image
                    cv2.circle(image, (int(tltrX), int(tltrY)), 5, (255, 0, 0), -1)
                    cv2.circle(image, (int(blbrX), int(blbrY)), 5, (255, 0, 0), -1)
                    cv2.circle(image, (int(tlblX), int(tlblY)), 5, (255, 0, 0), -1)
                    cv2.circle(image, (int(trbrX), int(trbrY)), 5, (255, 0, 0), -1)
                    cv2.circle(image, (int(x0), int(y0)), 5, (0, 255, 0), -1)


                    cv2.putText(image, "{:.1f}px;{:.1f}px".format(self.imshape_px[0], self.imshape_px[1]),
                                (int(0 + 15), int(0 + 20)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 0), 2)
                    cv2.putText(image, "{:.1f}mm;{:.1f}mm".format(self.imshape_mm[0], self.imshape_mm[1]),
                                ((0 + 15), int(0 + 50)), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 0), 2)
                    """
                    # draw the object sizes on the image
                    cv2.putText(image, "{:.1f}px;{:.1f}mm".format(dA, dimA),
                                (int(tltrX - 15), int(tltrY - 10)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.25, (255, 0, 0), 1)
                    cv2.putText(image, "{:.1f}px;{:.1f}mm".format(dB, dimB),
                                (int(trbrX + 10), int(trbrY)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.25, (255, 0, 0), 1)
                    # ------------------

    def cameraCallback(self, data):
        cvImage = utils.getCVImage(data)
        listObjects, imageSeeing = self.lookAt(cvImage)
        self.pubListObjects.publish(listObjects)
        imageSeeingMsg = utils.getMsgImage(imageSeeing)
        self.pubSeeing.publish()

    def cameraPositionCallback(self, pos, ori):
        rospy.loginfo("cameraPositionCallback is run!")
