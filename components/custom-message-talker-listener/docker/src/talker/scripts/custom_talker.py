#!/usr/bin/env python3
import rospy
import names
from share.msg import Person

def talker():
    pub = rospy.Publisher('custom_chatter', Person)
    rospy.init_node('custom_talker', anonymous=True)
    r = rospy.Rate(10) #10hz
    msg = Person()

    while not rospy.is_shutdown():
    	msg.first_name = names.get_first_name()
    	msg.last_name = names.get_last_name()
    	rospy.loginfo(msg)
    	pub.publish(msg)
    	r.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException: pass
