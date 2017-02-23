#!/usr/bin/env python
import sys
import rospy
from cvision.srv import CameraFrame

NODE_NAME = 'send_object_server'
SERVICE_NAME = 'send_object'

def sendObject(angle, position):
    rospy.wait_for_service(SERVICE_NAME)
    try:
        srvFound = True
        sendObject = rospy.ServiceProxy(SERVICE_NAME, CameraFrame)
        resp = sendObject(angle, position)
        return resp.state
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def usage():
    return "%s [0 1 0 7 7 7]"%sys.argv[0]

if __name__ == "__main__":
    if len(sys.argv) == 7:
        angle = [int(sys.argv[1]), int(sys.argv[2]), int(sys.argv[3])]
        position = [int(sys.argv[4]), int(sys.argv[5]), int(sys.argv[6])]
        else:
            print usage()
            sys.exit(1)
        print "Requesting %s+%s"%(angle, position)
        print sendObject(angle, position)

