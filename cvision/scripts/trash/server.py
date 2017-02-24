#!/usr/bin/env python
import rospy
import time
from cvision.srv import Camera
from cvision.msg import Object
from cvision.msg import ListObjects
from cvision.msg import Orientation


NODE_NAME = 'object_server'
SERVICE_NAME = 'send_object'

def handle(req):
    return req.state

def startCameraServer():
    print "Ready for send!"
    rospy.spin()


if __name__ == "__main__":
    CameraSever()