#!/usr/bin/env python
import argparse

import rospy

import camera_switch_server

CUSTOM_TOPIC = '/usb_cam/image_raw'
LENGTH = 0


def talker():
    ap = argparse.ArgumentParser()
    ap.add_argument('-l', '--length', required=True,
                    help='length from camera to floor  in [mm]')
    args = vars(ap.parse_args())
    LENGTH = float(args['length'])

    rospy.init_node('cv_recognizer', anonymous=False)
    # pycv_ok.Recognize(CUSTOM_TOPIC, LENGTH)

    camera_switch_server.CameraSwitchServer(CUSTOM_TOPIC, LENGTH)

    rospy.spin()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
