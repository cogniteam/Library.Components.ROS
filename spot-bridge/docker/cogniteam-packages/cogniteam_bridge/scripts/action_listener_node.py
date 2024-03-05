#!/usr/bin/env python

import rospy
from std_srvs.srv import Trigger
from std_msgs.msg import String

def action_callback(data):
    service_name = f"/spot/{data.data.lower()}"
    print(f'sending request to service: {service_name}')
    rospy.wait_for_service(service_name)
    try:
        service_proxy = rospy.ServiceProxy(service_name, Trigger)
        response = service_proxy()
        print(f"Service '{service_name}' activated successfully")
    except rospy.ServiceException as e:
        print(f"Service call to {service_name} failed: {e}")

def action_listener():
    print('Action listener node is active...')
    rospy.init_node('action_listener_node')
    rospy.Subscriber('/action', String, action_callback)
    rospy.spin()

if __name__ == '__main__':
    action_listener()