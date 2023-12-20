#!/usr/bin/env python3
import rospy
import tf
from geometry_msgs.msg import PoseStamped

if __name__ == '__main__':
    rospy.init_node('generic_robot_pose')

    # Get source and target frames from parameters
    source_frame = rospy.get_param("~source_frame", "/map")
    target_frame = rospy.get_param("~target_frame", "/base_link")

    listener = tf.TransformListener()
    pose_pub = rospy.Publisher('/robot_pose', PoseStamped, queue_size=10)

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            (trans, rot) = listener.lookupTransform(source_frame, target_frame, rospy.Time(0))
            print(str(rot))

            pose_msg = PoseStamped()
            pose_msg.header.stamp = rospy.Time.now()
            pose_msg.header.frame_id = source_frame
            pose_msg.pose.position.x = trans[0]
            pose_msg.pose.position.y = trans[1]
            pose_msg.pose.position.z = trans[2]
            pose_msg.pose.orientation.x = rot[0]
            pose_msg.pose.orientation.y = rot[1]
            pose_msg.pose.orientation.z = rot[2]
            pose_msg.pose.orientation.w = rot[3]

            pose_pub.publish(pose_msg)

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        rate.sleep()
