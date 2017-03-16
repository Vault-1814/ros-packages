#!/usr/bin/env python
import argparse
import rospy

import camera_switch_server

CUSTOM_TOPIC = '/usb_cam/image_rect'


def talker():
    """ the main launching function """
    ap = argparse.ArgumentParser()
    ap.add_argument('-l', '--length', required=True,
                    help='length from camera to surface in [mm]')
    args = vars(ap.parse_args())
    length = float(args['length'])

    rospy.init_node('cv_recognizer', anonymous=False)
    rospy.loginfo("Camera's topic is " + CUSTOM_TOPIC)
    camera_switch_server.CameraSwitchServer(CUSTOM_TOPIC, length)
    rospy.spin()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException, e:
        print("talker.py: " + e.message)
