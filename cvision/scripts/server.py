#!/usr/bin/env python

from cvision.srv import CameraFrame
import rospy

NODE_NAME = 'send_object_server'
SERVICE_NAME = 'send_object'

def handle(req):
    print(req.angle, req.position)
    return 1


def sendObjectData():
    rospy.init_node(NODE_NAME)
    s = rospy.Service(SERVICE_NAME, CameraFrame, handle)
    print "Ready for send!"
    rospy.spin()

if __name__ == "__main__":
    sendObjectData()