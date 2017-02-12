#!/usr/bin/env python
import rospy
import cv_methods_factory.exmodules.config_blurs as cb


def main():
    rospy.init_node('cv_tests', anonymous=False)
    cb.BlurConfig('/usb_cam/image_raw', 'preview')
    rospy.Rate(10)
    rospy.spin()
    #while not rospy.is_shutdown():
    #pass

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass