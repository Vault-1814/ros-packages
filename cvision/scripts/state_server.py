#!/usr/bin/env python
import rospy
import time

from std_msgs.msg import Float64
from cvision.srv import State
from cvision.msg import Object
from cvision.msg import ListObjects
from cvision.msg import Orientation

NODE_NAME = 'state'
SERVICE_NAME = 'state'


class CameraSever:
    def __init__(self, length):
        # rospy.init_node(NODE_NAME)
        self.length = length

        # publish orientation for starting work cv
        self.pubOrientation = rospy.Publisher('orientation', Orientation, queue_size=1)

        self.srvMState = rospy.Service(SERVICE_NAME, State, self.handle)
        self.startCameraServer()

    def handle(self, req):
        msg = Orientation()
        if req.state == 1:
            msg.length = self.length
        else:
            msg.length = -1
        self.pubOrientation.publish(msg)
        return req.state

    def startCameraServer(self):
        print "ready for start! get me '1' to state service, pleeeease!\n\tor '0' for stop"
        rospy.spin()


if __name__ == "__main__":
    CameraSever(520.0)
