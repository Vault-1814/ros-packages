#!/usr/bin/env python
import rospy
import cv_methods_factory.fov as fov
import camera_factory.camera_source as camsrc


def talker():
    rospy.init_node('cv_recognizer', anonymous=False)
    cameraFactory = camsrc.CameraFactory()
    camera = cameraFactory.getCameraSource(camsrc.SOURCE_OPENNI2)
    cameraTopic = camera.getTopic()
    cameraPositionTopic = 'position'
    # main callback there
    rospy.loginfo(cameraTopic)
    rospy.loginfo(cameraPositionTopic)
    fov.FovVision(cameraTopic, cameraPositionTopic)
    rospy.Rate(10)
    rospy.spin()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
