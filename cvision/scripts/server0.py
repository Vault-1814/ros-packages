#!/usr/bin/env python
import rospy
import time
from cvision.srv import Camera
from cvision.msg import Object
from cvision.msg import ListObjects
from cvision.msg import Orientation


NODE_NAME = 'object_server'
SERVICE_NAME = 'send_object'


class CameraSever:
	def __init__(self):
		self.subListObjects = rospy.Subscriber('/list_objects',ListObjects, callback, queue_size=1)
		self.pubOrientation = rospy.Publisher('orientation', Orientation, queue_size=1)
	    rospy.init_node(NODE_NAME)
	    self.srvCameraSrv = rospy.Service(SERVICE_NAME, Camera, self.handle)
		startCameraServer()

		self.selectedObject = None

	def callback(data):
		if len(data.list) > 0:
			# the place of selecting object :)
			for obj in data.list:
				if obj.shape == ''
			self.selectedObject = data.list[0]

	def handle(req):
		if req.command == 1:
			msg = Orientation()
			msg.length = req.length
			self.pubOrientation.pub(msg)
			print(self.selectedObject)
			if self.selectedObject.
	    return object

	def startCameraServer():
	    print "Ready for send!"
	    rospy.spin()


if __name__ == "__main__":
    CameraSever()