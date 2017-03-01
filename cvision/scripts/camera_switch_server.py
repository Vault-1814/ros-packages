#!/usr/bin/env python
import time

import rospy

from cvision.srv import CameraListObjects
from sensor_msgs.msg import Image

import libs.utils as u
import pymeasuring

NODE_NAME = 'cv_switch'
SERVICE_NAME = 'get_list_objects'


class CameraSwitchServer:
    def __init__(self, source, length):
        self.isReady = False
        self.source = source
        self.length = length

        self.imageInfo = {}
        self.imageInfo['shape'] = None
        self.imageInfo['ratio'] = None
        self.measuring = None

        self.list = None
        self.subCamera = None

        self.pub_view_main = rospy.Publisher('see_main', Image, queue_size=1)

        self.srvMState = rospy.Service(SERVICE_NAME, CameraListObjects, self.handle)
        self.startCameraServer()

    def init(self):
        imageDim = u.getDimImage(self.length, 0, 0, 78)  # 54.5, 42.3, 66.17
        self.imageInfo['ratio'] = u.getRatio(self.imageInfo['shape'],
                                             imageDim)
        rospy.loginfo('Computed ratios are ' + str(self.imageInfo))
        self.measuring = pymeasuring.Measuring(self.imageInfo)
        rospy.loginfo('Measuring init complete.')

    def cameraCallback(self, data):
        cvImage, self.imageInfo['shape'] = u.getCVImage(data)
        if self.measuring is not None:
            self.list, imageex, self.isReady = self.measuring.getListObjects(cvImage)

            # message for see result
            msg_image = u.getMsgImage(imageex)
            self.pub_view_main.publish(msg_image)
        else:
            if self.imageInfo['shape'] is not None:
                self.init()
            else:
                rospy.logerr('imageInfo.shape in None!!1!')

    def handle(self, req):
        self.subCamera = rospy.Subscriber(self.source, Image,
                                          self.cameraCallback)
        while not self.isReady:
            time.sleep(0.01)
        self.subCamera = None
        return [self.list]

    def startCameraServer(self):
        print "ready for start! get me '1' to state service, pleeeease!\n\tor '0' for stop"
        rospy.spin()    # hmmmmm...


if __name__ == "__main__":
    CameraSwitchServer(520.0)
