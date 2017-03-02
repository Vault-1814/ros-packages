#!/usr/bin/env python
import argparse
import rospy

import camera_switch_server

CUSTOM_TOPIC = '/usb_cam/image_rect'


def talker():
    ap = argparse.ArgumentParser()
    ap.add_argument('-l', '--length', required=True,
                    help='length from camera to floor  in [mm]')
    args = vars(ap.parse_args())
    length = float(args['length'])

    rospy.init_node('cv_recognizer', anonymous=False)
    rospy.loginfo("camera's topic is " + CUSTOM_TOPIC)
    rospy.loginfo("length [mm] from camera to floor is " + str(length))
    # pycv_ok.Recognize(CUSTOM_TOPIC, length)
    camera_switch_server.CameraSwitchServer(CUSTOM_TOPIC, length)
    rospy.spin()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
