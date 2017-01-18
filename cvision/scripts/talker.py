#!/usr/bin/env python
import roslib
import rospy
from cvision.msg import Object
from cvision.msg import ListObjects


def getMsg():
    """there are processing camera frames and recognizing objects"""
    o1 = Object()
    o1.shape = 'cude'
    o1.coordinates = (10, 10, 0)
    o1.dimensions = (10, 10, 10)
    o2 = Object()
    o2.shape = 'sphere'
    o2.coordinates = (50, 10, 0)
    o2.dimensions = (10, 10, 10)

    msg = ListObjects()
    l = []
    l.append(o1)
    l.append(o2)
    msg = l

    return msg


def talker():
    rospy.init_node('cv_recognizer', anonymous=False)
    pub = rospy.Publisher('cv_broadcast', ListObjects, queue_size=10)
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        msg = getMsg()
        rospy.loginfo('I sent %s objects to cv_broadcast topic', len(msg))
        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
