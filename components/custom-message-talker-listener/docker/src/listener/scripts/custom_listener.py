#!/usr/bin/env python3
import rospy
from share.msg import Person

def callback(data):
    rospy.loginfo("got person with first name: %s & last name: %s" % (data.first_name, data.last_name))

def listener():
    rospy.init_node('custom_listener', anonymous=True)
    rospy.Subscriber("custom_chatter", Person, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()