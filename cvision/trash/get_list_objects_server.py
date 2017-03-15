#!/usr/bin/env python
import rospy
import time

from std_srvs.srv import Empty
from std_msgs.msg import Float64
from cvision.srv import State
from cvision.msg import Object
from cvision.msg import ListObjects
from cvision.msg import Orientation

NODE_NAME = 'cv_switch'
SERVICE_NAME = 'get_list_objects'


class CameraListObjectsSever:
    def __init__(self, source, length):
        # rospy.init_node(NODE_NAME)

        self.imageInfo = {}
        self.imageInfo['shape'] = None
        self.imageInfo['ratio'] = None
        self.measuring = None

        self.subCamera = rospy.Subscriber(source, Image,
                                          self.cameraCallback)

        self.srvMState = rospy.Service(SERVICE_NAME, CameraListObjects, self.handle)
        self.startCameraListObjectsSever()

    def handle(self, req):

        return list

    def startCameraListObjectsSever(self):
        print "CameraListObject server is ready! Get me empty message :)"
        rospy.spin()


if __name__ == "__main__":
    CameraListObjectsSever()
