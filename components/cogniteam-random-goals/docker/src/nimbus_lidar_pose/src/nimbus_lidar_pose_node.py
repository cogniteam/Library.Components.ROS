#!/usr/bin/env python3  
import rospy
import math
import tf
import geometry_msgs.msg
from geometry_msgs.msg import PoseStamped


if __name__ == '__main__':
    rospy.init_node('nimbus_lidar_pose')

    listener = tf.TransformListener()
    
    nimbus_lidar_pose_pub = rospy.Publisher('/nimbus_lidar_pose', PoseStamped, queue_size=10)   

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            (trans,rot) = listener.lookupTransform('map', 'nimbus/slamtec-rplidar-a3', rospy.Time(0))
            print(str(rot))

            lidar_pose = PoseStamped()
            lidar_pose.header.stamp = rospy.Time.now()
            lidar_pose.header.frame_id = "map"
            lidar_pose.pose.position.x = trans[0]
            lidar_pose.pose.position.y = trans[1]
            lidar_pose.pose.position.z = trans[2]
           
            lidar_pose.pose.orientation.x = rot[0]
            lidar_pose.pose.orientation.y = rot[1]
            lidar_pose.pose.orientation.z = rot[2]
            lidar_pose.pose.orientation.w = rot[3]

            nimbus_lidar_pose_pub.publish(lidar_pose)

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

      

        rate.sleep()