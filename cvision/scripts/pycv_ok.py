#!/usr/bin/env python
# -*- coding: utf-8
import rospy
from cvision.msg import ListObjects
from cvision.msg import Orientation
from sensor_msgs.msg import Image

import libs.utils as u
from camera_switch_server import CameraSwitchServer
from pymeasuring import Measuring


class Recognize:

    def __init__(self, source, length, detail=True):
        rospy.logerr("OK, i started!")

        self.imageInfo = {}
        self.imageInfo['shape'] = None
        self.imageInfo['ratio'] = None
        self.measuring = None

        self.subCamera = rospy.Subscriber(source, Image,
                                          self.cameraCallback)
        self.subOrientation = rospy.Subscriber('/orientation',
                                               Orientation,
                                               self.orientationCallback,
                                               queue_size=1)

        self.pub_main = rospy.Publisher('list_objects', ListObjects, queue_size=1)
        self.pub_view_main = rospy.Publisher('see_main', Image, queue_size=1)
        # server wait
        self.stateServer = CameraSwitchServer(length)

    def init(self):
        self.measuring = Measuring(self.imageInfo)
        rospy.loginfo('Measuring init complete.')
        print(self.imageInfo)

    def cameraCallback(self, data):
        # TODO глупо вычислять все время shape !!!
        cvImage, self.imageInfo['shape'] = u.getCVImage(data)
        if self.measuring is not None:
            list, imageex = self.measuring.getListObjects(cvImage)

            "for topic sending"
            # # message for further work
            # msg = list
            # rospy.loginfo('Send %s objects', len(msg))
            # self.pub_main.publish(msg)

            "for service sending"

            # message for see result
            msg_image = u.getMsgImage(imageex)
            self.pub_view_main.publish(msg_image)
        else:
            if self.imageInfo['ratio'] is not None:
                self.init()

    def orientationCallback(self, data):
        if data.length > 0:
            l = data.length
            imageDim = u.getDimImage(l, 0, 0, 78) # 54.5, 42.3, 66.17
            if self.imageInfo['shape'] is not None:
                self.imageInfo['ratio'] = u.getRatio(self.imageInfo['shape'], imageDim)
                rospy.loginfo('Computed ratios are ' + str(self.imageInfo['ratio']))
        else:
            rospy.logerr('SEEING WAS STOPED')
            self.imageInfo['shape'] = None
            self.imageInfo['ratio'] = None
            self.measuring = None

