#!/usr/bin/env python3
from __future__ import print_function
 
from services.srv import greeter,greeterResponse
import rospy

def greet(req):
    print("[Greeting %s %s]"%(req.first_name,req.last_name))
    return greeterResponse('hello %s %s'%(req.first_name,req.last_name))

def greeting_server():
    rospy.init_node('greeter_server')
    s = rospy.Service('greeter', greeter, greet)
    print("Ready to get requests.")
    rospy.spin()

if __name__ == "__main__":
    greeting_server()