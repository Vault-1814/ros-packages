import rospy
import cv2
import math
import numpy as np
from imutils import perspective
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from cvision.msg import Object
from cvision.msg import ListObjects
from cvision.msg import Orientation
from scipy.spatial import distance as dist


class Recognize:

    def __init__(self, source, is_ros_msg=False):
        self.is_ros_msg = is_ros_msg

        self.pixelsPerMetric = 1
        self.WIDTH_PIXEL = 85 # nexus 5 1920x1080

        self.imshape_px = [0, 0, 0]
        self.imshape_mm = self.getWHImage(420, 0, 0, 58)

        global DISSIMILARITY_THRESHOLD
        DISSIMILARITY_THRESHOLD = 1

        self.subscriber_camera = rospy.Subscriber(source, Image,
                                                  self.cameraCallback)
        self.subscriber_orientation = rospy.Subscriber('/orientation',
                                                       Orientation,
                                                       self.orientationCallback,
                                                       queue_size=1)

        self.pub_main = rospy.Publisher('list_objects', ListObjects, queue_size=1)
        self.pub_view_main = rospy.Publisher('see_main', Image, queue_size=1)

    def midpoint(self, ptA, ptB):
        return (ptA + ptB) / 2

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
        # """ Returns the unit vector of the vector.  """
        return vector / np.linalg.norm(vector)

    def angle_between(self, v1, v2):
        v1_u = self.unit_vector(v1)
        v2_u = self.unit_vector(v2)
        return np.arccos(np.clip(np.dot(v1_u, v2_u), -1.0, 1.0))

    def getWHImage(self, l, hfov=58, vfov=45, dfov=70):
        # """
        #     hfov and vfov for asus xtion pro live
        #     returns width and height of an image in [mm]
        # """
        hfov_rad = hfov * math.pi / 180
        vfov_rad = vfov * math.pi / 180
        dfov_rad = dfov * math.pi / 180
        width = 2 * l * math.tan(hfov_rad / 2)
        height = 2 * l * math.tan(vfov_rad / 2)
        diag = 2 * l * math.tan(dfov_rad / 2)
        return width, height, diag

    def getListObjects(self, image, detail=True):
        list_obj = []

        # rospy.loginfo(image.shape)

        # Blur
        blur = 3
        blurred = cv2.medianBlur(image, blur)

        # Grayscale
        gray = cv2.cvtColor(blurred, cv2.COLOR_BGR2GRAY)

        # Invert
        if cv2.getTrackbarPos('invert', 'Bars'):
            gray = cv2.bitwise_not(gray)

        # Threshold
        # threshold_min = cv2.getTrackbarPos('threshold_min', 'Bars')
        threshold_min = 100
        _, blurred = cv2.threshold(gray, threshold_min, 255, cv2.THRESH_BINARY)

        # Edges
        # sigma = cv2.getTrackbarPos('sigma', 'Bars') / 100.
        v = np.median(image)
        sigma = 0.33
        canny_low = int(max(0, (1 - sigma) * v))
        canny_high = int(min(255, (1 + sigma) * v))
        edged = cv2.Canny(blurred, canny_low, canny_high)
        edged = cv2.dilate(edged, None, iterations=3)
        edged = cv2.erode(edged, None, iterations=2)

        screw_contour = np.array([[[43, 17]], [[31, 24]], [[27, 33]], [[85, 183]], [[75, 194]], [[84, 217]], [[129, 206]], [[139, 197]], [[136, 180]], [[132, 174]], [[123, 175]], [[118, 169]], [[61, 20]], [[56, 16]]])

        # Find contours
        contours, hierarchy = cv2.findContours(edged, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        obj_screw = None
        draw_contours = []  # (ret, contour)

        for contour in contours:
            if cv2.contourArea(contour) < 600:
                continue
            ret = cv2.matchShapes(contour, screw_contour, 1, 0)
            if obj_screw is None or ret < obj_screw[0]:
                obj_screw = (ret, contour)
                if detail:
                    draw_contours.append((ret, contour))

        if obj_screw[0] < DISSIMILARITY_THRESHOLD:

            # rospy.loginfo("find something")

            obj = Object()

            box = cv2.minAreaRect([1])
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
            # """
            #     Compute the size of the object.
            #     object_diag_mm = object_diag_px * frame_mm / frame_px
            #     !!! because angle for horizontal and vertical fields of view are different
            #             and we do not know orientation object (still)
            # """
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

            # rospy.loginfo(str(int(dA)) + ' ' + str(int(dB)))
            if dA >= dB:
                aVector = (tltrX - blbrX, tltrY - blbrY)
                angle = -self.angle_between(xVector, aVector)
            else:
                bVector = (tlblX - trbrX, tlblY - trbrY)
                angle = -self.angle_between(xVector, bVector)

            if abs(angle) > math.pi / 2:
                angle += math.pi

            obj.shape = 'screw'
            obj.dimensions = (dimA*0.001, dimB*0.001, 1)
            rospy.loginfo(obj.dimensions)
            obj.coordinates_center_frame = (ycc*ratio*0.001, xcc*ratio*0.001, 0)
            rospy.loginfo(obj.coordinates_center_frame)
            # obj.orientation = (0, angle, 0)

            # sendObject(obj.orientation, obj.coordinates_center_frame)

            list_obj.append(obj)

        if detail:
            for ret, contour in draw_contours:
                M = cv2.moments(contour)
                text_pos = (int(M['m10'] / M['m00']),
                            int(M['m01'] / M['m00'])) if M['m00'] else (0, 0)

                mask = np.zeros(gray.shape, np.uint8)
                cv2.drawContours(mask, [contour], 0, 255, -1)
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

        # edged_bgr = cv2.cvtColor(edged, cv2.COLOR_GRAY2BGR)

        if detail:
            return list_obj, image
        else:
            return list_obj




    def cameraCallback(self, data):
        if self.is_ros_msg:
            cv_image = self.getCVImage(data)
        else:
            cv_image = data
        list_obj, imageex = self.getListObjects(cv_image, True)
        # message for futher work
        msg = ListObjects()
        msg = list_obj
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