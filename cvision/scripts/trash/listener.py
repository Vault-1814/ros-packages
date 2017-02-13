#!/usr/bin/env python
import roslib
import rospy
from cvision.msg import Object
from cvision.msg import ListObjects


def callback(data):
    """data.list contains list of objects from the table"""
    list = data.list
    log_string = ''
    for o in list:
        log_string += o.shape + '; ';
    rospy.loginfo('I got %s objects: %s', len(list), log_string)


def listener():
    rospy.init_node('manipulator', anonymous=True)
    rospy.Subscriber('cv_broadcast', ListObjects, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
