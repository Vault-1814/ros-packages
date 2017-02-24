import rospy
import cv2
import numpy as np
import math
from scripts.client import *
from cv_bridge import CvBridge, CvBridgeError
from scipy.spatial import distance as dist
from imutils import perspective
from sensor_msgs.msg import Image
from cvision.msg import Object
from cvision.msg import ListObjects
from cvision.msg import Orientation


class Recognize:

    def __init__(self, source, is_ros_msg=False):
        self.is_ros_msg = is_ros_msg

        self.pixelsPerMetric = 1
        self.WIDTH_PIXEL = 85 # nexus 5 1920x1080

        self.imshape_px = [0, 0, 0]
        self.imshape_mm = [0, 0, 0]

        self.subscriber_camera = rospy.Subscriber(source, Image,
                                                  self.cameraCallback)
        self.subscriber_orientation = rospy.Subscriber('/orientation',
                                                       Orientation,
                                                       self.orientationCallback,
                                                       queue_size=1)

        self.pub_main = rospy.Publisher('list_objects', ListObjects, queue_size=1)
        self.pub_view_main = rospy.Publisher('see_main', Image, queue_size=1)

    def getWHImage(self, l, hfov=58, vfov=45, dfov=70):
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
            diag_px = math.sqrt(cv_image.shape[0] ** 2 + cv_image.shape[1] ** 2)
            self.imshape_px[0], self.imshape_px[1], self.imshape_px[2] = (cv_image.shape[0], cv_image.shape[1], diag_px)
        except CvBridgeError, e:
            rospy.loginfo("Conversion failed: %s", e.message)
        return cv_image

    def getMsgImage(self, cv_image):
        bridge = CvBridge()
        msg_image = None
        try:
            msg_image = bridge.cv2_to_imgmsg(cv_image, "bgr8")
            #msg_image = bridge.cv2_to_imgmsg(cv_image, "8UC1")
        except CvBridgeError, e:
            rospy.loginfo("Conversion failed: %s", e.message)
        return msg_image

    def unit_vector(self, vector):
        """ Returns the unit vector of the vector.  """
        return vector / np.linalg.norm(vector)

    def angle_between(self, v1, v2):
        v1_u = self.unit_vector(v1)
        v2_u = self.unit_vector(v2)
        return np.arccos(np.clip(np.dot(v1_u, v2_u), -1.0, 1.0))

    def getListObjects(self, image, detail=False):
        """ finds objects on the image, analizing it, push to the list
            if canvas is None:
                list = getListObjects(image)
            else if canvas:
                list, imageex= getListObjects(image, true)
                    where on the canvas would be drawn contours and etc.
        """
        list = []
        edged = cv2.Canny(image, 70, 190)
        edged = cv2.dilate(edged, None, iterations=3)
        edged = cv2.erode(edged, None, iterations=2)
        contours, hierarchy = cv2.findContours(edged, cv2.RETR_EXTERNAL,
                                               cv2.CHAIN_APPROX_SIMPLE)
        for contour in contours:
            o = Object()
            if cv2.contourArea(contour) < 600:
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
            xcc = x0 - xoc
            ycc = y0 - yoc
            #rospy.loginfo("O(%s; %s) --> O'(%s; %s)", tltrX, tlblY, xcc, ycc)
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
            alpha = math.atan2(dB, dA)
            dimA = dimD * math.cos(alpha)
            dimB = dimD * math.sin(alpha)

            # orientation object
            xVector = (0, -self.imshape_px[0])
            amax, bmax = '', ''

            aAngle, bAngle = 0, 0

            rospy.loginfo(str(int(dA)) + ' ' + str(int(dB)))
            if dA >= dB:
                aVector = (tltrX - blbrX, tltrY - blbrY)
                angle = -self.angle_between(xVector, aVector)
            else:
                bVector = (tlblX - trbrX, tlblY - trbrY)
                angle = -self.angle_between(xVector, bVector)

            if abs(angle) > math.pi / 2:
                angle += math.pi

            # ***
            # TODO detecting and analezing objects
            o.shape = 'undefined'
            o.dimensions = (dimA*0.001, dimB*0.001, 1)
            o.coordinates_center_frame = (ycc*ratio*0.001, xcc*ratio*0.001, 0)
            o.orientation = (0, angle, 0)

            sendObject(o.orientation, o.coordinates_center_frame)
            #info = "Height:%d\tWidth:%d\n" % (dimA, dimB)
            #rospy.loginfo(info)
            list.append(o)
            # ***
            if detail:
                # ------------------
                # draw on source frame
                cv2.drawContours(image, [box.astype("int")], -1, (0, 255, 0), 1)
                # draw corner points
                for (x, y) in box:
                    # top left corner of a object
                    cv2.circle(image, (int(tl[0]), int(tl[1])), 5, (255, 0, 0), 2)

                    cv2.putText(image, "{0:.1f}".format(angle*180/math.pi),
                                (int(0 + 50), int(0 + 25)),
                                cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

                    # BLUE point in a center frame
                    cv2.circle(image, (int(x0), int(y0)), 5, (0, 0, 255), 2)
                    #center of object in center coordinate system
                    #cv2.circle(image, (int(xcc), int(ycc)), 5, (0, 255, 0), 2)
                    # red point of a center object
                    cv2.circle(image, (int(xoc), int(yoc)), 5, (255, 0, 0), 2)

                    # cross in a center frame
                    cv2.line(image, (0, y0), (self.imshape_px[1], y0), (0, 0, 255), 1)
                    cv2.line(image, (x0, 0), (x0, self.imshape_px[0]), (0, 0, 255), 1)

                    # cross in a center object
                    # dA
                    cv2.line(image, (int(tltrX), int(tltrY)), (int(blbrX), int(blbrY)), (255, 0, 255), 1)
                    # dB
                    cv2.line(image, (int(tlblX), int(tlblY)), (int(trbrX), int(trbrY)), (255, 0, 255), 1)

                    cv2.putText(image, 'A',
                                (int(tltrX), int(tltrY)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
                    cv2.putText(image, 'B',
                                (int(tlblX), int(tlblY)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
                    cv2.putText(image, 'X',
                                (int(x0), int(15)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

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

                    # draw the object sizes on the image
                    cv2.putText(image, "{:.1f}px;{:.1f}mm".format(dA, dimA), (int(tltrX - 15), int(tltrY - 10)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.25, (255, 0, 0), 1)
                    cv2.putText(image, "{:.1f}px;{:.1f}mm".format(dB, dimB), (int(trbrX + 10), int(trbrY)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.25, (255, 0, 0), 1)
                    """
                    
        if not detail:
            return list
        else:
            return list, image

    def cameraCallback(self, data):
        if self.is_ros_msg:
            cv_image = self.getCVImage(data)
        else:
            cv_image = data
        list, imageex = self.getListObjects(cv_image, True)
        # message for futher work
        msg = ListObjects()
        msg = list
        rospy.loginfo('Send %s objects', len(msg))
        self.pub_main.publish(msg)
        # message for see result
        msg_image = self.getMsgImage(imageex)
        self.pub_view_main.publish(msg_image)


    def orientationCallback(self, data):
        l = data.length
        width_mm, height_mm, diag_mm = self.getWHImage(l, 54.5, 42.3, 66.17)
        #rospy.loginfo("im_diag_px = %s", self.imshape_px[2])
        #rospy.loginfo("im_diag_mm = %s", diag_mm)
        self.imshape_mm = (width_mm, height_mm, diag_mm)
