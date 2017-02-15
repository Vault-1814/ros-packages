#!/usr/bin/env python
import rospy
import cv_methods_factory.exmodules.test_filters as tf


def main():
    rospy.init_node('cfg_filtering', anonymous=False)
    tf.CfgFilteringNode('/usb_cam/image_rect', 'preview')
    rospy.Rate(10)
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
