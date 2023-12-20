#!/usr/bin/env python3

from __future__ import print_function

import sys
from urllib import response
import rospy
from services.srv import *
import names
import time

def greet_client(first_name, last_name):
    rospy.wait_for_service('greeter')
    try:
        greetingServer = rospy.ServiceProxy('greeter', greeter)
        response = greetingServer(first_name, last_name)
        return response
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def usage():
    return "%s [first_name last_name]"%sys.argv[0]

if __name__ == "__main__":
    while not rospy.is_shutdown():
        first_name = names.get_first_name()
        last_name = names.get_last_name()
        rospy.loginfo("sending request: {first_name\:%s, last_name\:%s}"%(first_name, last_name))
        response = greet_client(first_name, last_name)
        rospy.loginfo("got response: %s"%response.greeting)
        time.sleep(0.2)

    rospy.spin()