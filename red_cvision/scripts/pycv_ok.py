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
    """ older version """

    def __init__(self, source, length, detail=True):
        rospy.logerr("OK, i started!")

        self.imageInfo = dict()
        self.imageInfo['shape'] = None
        self.imageInfo['ratio'] = None
        self.measuring = None

        self.subCamera = rospy.Subscriber(source, Image,
                                          self.cameraCallback)

        self.pub_main = rospy.Publisher('list_objects', ListObjects, queue_size=1)
        self.pub_view_main = rospy.Publisher('see_main', Image, queue_size=100)
        # server wait
        self.stateServer = CameraSwitchServer(length)

    def init(self):
        self.measuring = Measuring(self.imageInfo)
        rospy.loginfo('Measuring init complete.')
        print(self.imageInfo)

    def cameraCallback(self, data):
        # TODO глупо вычислять все время shape !!!
        image, self.imageInfo['shape'] = u.getCVImage(data)
        if self.measuring is not None:
            list, image = self.measuring.getListObjects(image)
        else:
            if self.imageInfo['ratio'] is not None:
                self.init()
        # message for see result
        msg_image = u.getMsgImage(image)
        self.pub_view_main.publish(msg_image)


