#!/usr/bin/env python
import rospy
import time

from cvision.srv import CameraListObjects
from sensor_msgs.msg import Image

import libs.utils as u
import pymeasuring

NODE_NAME = 'cv_switch'
SERVICE_NAME = 'get_list_objects'


class CameraSwitchServer:
    """ The server for communication with manipulation node """

    def __init__(self, source, length):
        self.isReady = False    # while False then objects don't find :")

        self.source = source
        self.length = length

        self.imageInfo = dict()
        self.imageInfo['shape'] = None
        self.imageInfo['ratio'] = None  # ration [mm] / [px]

        self.measuring = None

        self.list = None                # list of found objects
        self.subCamera = None

        # '/see_main' topic provides preview of processed image
        self.pub_view_main = rospy.Publisher('see_main', Image, queue_size=1)

        # server for requests for found objects
        self.srvMState = rospy.Service(SERVICE_NAME, CameraListObjects, self.handle)

        # start ros
        self.startCameraServer()

    def init(self):
        """ initialization of need parameters"""
        imageDim = u.getDimImage(self.length, 0, 0, 78)  # 54.5, 42.3, 66.17
        self.imageInfo['ratio'] = u.getRatio(self.imageInfo['shape'],
                                             imageDim)

        self.measuring = pymeasuring.Measuring(self.imageInfo, self.length)

        # rospy.loginfo("dims of image [mm]: " + str(imageDim))
        # rospy.loginfo("ratios [mm/px]: " + str(self.imageInfo['ratio']))
        # rospy.loginfo("shape [px]: " + str(self.imageInfo['shape']))
        rospy.loginfo('init of measuring object is complete.')

    def cameraCallback(self, data):
        """ callback for image processing"""
        if not self.isReady:
            cvImage, self.imageInfo['shape'] = u.getCVImage(data)
            if self.measuring is not None:
                self.list, cvImage, self.isReady = self.measuring.getListObjects(cvImage)
                # preview topic /see_main
                msg_image = u.getMsgImage(cvImage)
                self.pub_view_main.publish(msg_image)
            else:
                if self.imageInfo['shape'] is not None:
                    self.init()
                else:
                    rospy.logerr("no video stream. check camera's topic!")

    def handle(self, req):
        """ handle of server's request """
        imAgree = False

        self.isReady = False
        self.subCamera = rospy.Subscriber(self.source, Image,
                                          self.cameraCallback)
        while not self.isReady:
            time.sleep(0.01)

        # checking for adequacy
        print(self.list)
        # flag = 0
        # while not imAgree:
        #     self.subCamera = None
        #     self.measuring = None
        #     flag = input('checking: type 1 if all right: ')
        #     if flag == 1:
        #         imAgree = True

        # stop processing image
        self.subCamera = None
        self.measuring = None
        return [self.list]

    def startCameraServer(self):
        rospy.loginfo('CV ready to start! Waiting msg...')
        rospy.spin()
