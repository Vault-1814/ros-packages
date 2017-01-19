import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from scipy.spatial import distance as dist
from imutils import perspective
from sensor_msgs.msg import Image
from cvision.msg import Object
from cvision.msg import ListObjects


class Recognize:

    def __init__(self, source):
        self.pixelsPerMetric = 1
        self.WIDTH_PIXEL = 85 # nexus 5 1920x1080

        self.imshape = (0, 0, 0)

        self.subscriber = rospy.Subscriber(source, Image, self.callback, queue_size=1)

        self.pub_main = rospy.Publisher('list_objects', ListObjects, queue_size=1)
        self.pub_view_main = rospy.Publisher('see_main', Image, queue_size=1)
        self.pub_view_depth = rospy.Publisher('see_depth', Image, queue_size=1)

    def midpoint(self, ptA, ptB):
        return (ptA[0] + ptB[0]) * 0.5, (ptA[1] + ptB[1]) * 0.5

    def clearCanvas(self, canvas):
        canvas[:, :, :] = (255, 255, 255)
        return canvas

    def getCanvas(self, h, w):
        canvas = np.zeros((h, w, 3), dtype=np.uint8)
        canvas = self.clearCanvas(canvas)
        return canvas

    def getCVImage(self, data):
        bridge = CvBridge()
        cv_image = None
        try:
            cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError, e:
            rospy.loginfo("Conversion failed: %s", e.message)
        return cv_image

    def getMsgImage(self, cv_image):
        bridge = CvBridge()
        msg_image = None
        try:
            msg_image = bridge.cv2_to_imgmsg(cv_image, "bgr8")
        except CvBridgeError, e:
            rospy.loginfo("Conversion failed: %s", e.message)
        return msg_image

    def getListObjects(self, image, detail=False):
        """ finds objects on the image, analizing it, push to the list
            if canvas is None:
                list = getListObjects(image)
            else if canvas:
                list, canvas = getListObjects(image, canvas)
                    where on the canvas would be drawn contours and etc.
        """
        list = []
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        edged = cv2.Canny(gray, 70, 120)
        edged = cv2.dilate(edged, None, iterations=3)
        edged = cv2.erode(edged, None, iterations=2)
        contours, hierarchy = cv2.findContours(edged, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
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
            if self.pixelsPerMetric is None:
                self.pixelsPerMetric = dB / self.WIDTH_PIXEL
            # compute the size of the object
            # TODO computing ratio [pixel / mm]
            dimA = dA / self.pixelsPerMetric
            dimB = dB / self.pixelsPerMetric
            # ***
            # TODO detecting and analezing objects
            o.shape = 'undefined'
            o.dimensions = (dimA, dimB, 1)
            o.coordinates = (0, 0, 0)
            #info = "Height:%d\tWidth:%d\n" % (dimA, dimB)
            #rospy.loginfo(info)
            list.append(o)
            # ***
            if detail:
                # ------------------
                # draw on source frame
                cv2.drawContours(image, [box.astype("int")], -1, (0, 255, 0), 2)
                # draw corner points
                for (x, y) in box:
                    cv2.circle(image, (int(x), int(y)), 5, (0, 0, 255), -1)
                    # draw the midpoints on the image
                    cv2.circle(image, (int(tltrX), int(tltrY)), 5, (255, 0, 0), -1)
                    cv2.circle(image, (int(blbrX), int(blbrY)), 5, (255, 0, 0), -1)
                    cv2.circle(image, (int(tlblX), int(tlblY)), 5, (255, 0, 0), -1)
                    cv2.circle(image, (int(trbrX), int(trbrY)), 5, (255, 0, 0), -1)
                    # draw lines between the midpoints
                    cv2.line(image, (int(tltrX), int(tltrY)), (int(blbrX), int(blbrY)), (255, 0, 255), 2)
                    cv2.line(image, (int(tlblX), int(tlblY)), (int(trbrX), int(trbrY)), (255, 0, 255), 2)
                    # draw the object sizes on the image
                    cv2.putText(image, "{:.1f}".format(dimA), (int(tltrX - 15), int(tltrY - 10)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2)
                    cv2.putText(image, "{:.1f}".format(dimB), (int(trbrX + 10), int(trbrY)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2)
                    # ------------------
        if not detail:
            return list
        else:
            return list, image

    def callback(self, data):
        cv_image = self.getCVImage(data)
        list, imageex = self.getListObjects(cv_image, True)
        # message for futher work
        msg = ListObjects()
        msg = list
        rospy.loginfo('Send %s objects', len(msg))
        self.pub_main.publish(msg)
        # message for see result
        # for instance:
        #   rosrun image_view image_view image:=/see_main
        msg_image = self.getMsgImage(imageex)
        self.pub_view_main.publish(msg_image)


