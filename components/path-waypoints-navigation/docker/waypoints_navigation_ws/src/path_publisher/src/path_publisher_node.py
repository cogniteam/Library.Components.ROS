#!/usr/bin/env python3
import rospy
import sys
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import String
from rospy import Time
import geometry_msgs.msg
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool



class path_publisher:

  def __init__(self):
    self.waypointsList = []
    self.sub = rospy.Subscriber("/move_base_simple/goal",PoseStamped,self.poseCallback, queue_size = 1)
    self.f = open('path.txt','w+')

  
  def poseCallback(self,pose_msg):
      poseStamped = PoseStamped()
      poseStamped.pose.position.x = pose_msg.pose.position.x
      poseStamped.pose.position.y = pose_msg.pose.position.y
      poseStamped.pose.position.z = pose_msg.pose.position.z
      poseStamped.header.frame_id = "map"
      poseStamped.header.stamp = rospy.Time(0)
      poseStamped.pose.orientation = pose_msg.pose.orientation
      orientation_list = [poseStamped.pose.orientation.x,poseStamped.pose.orientation.y,poseStamped.pose.orientation.z,poseStamped.pose.orientation.w]
      

      line = str(poseStamped.pose.position.x) + "," +  str(poseStamped.pose.position.y) + "," +  str(poseStamped.pose.position.z) + "," +  str(poseStamped.pose.orientation.x) + "," + str(poseStamped.pose.orientation.y) + "," + str(poseStamped.pose.orientation.z) + "," + str(poseStamped.pose.orientation.w)
      self.f.write(line + "\n")

    
def publishPath():
    path_pub = rospy.Publisher('/waypoints_route', Path, queue_size=10)
    route = Path()
    route.header.frame_id = "map"
    route.header.stamp = rospy.Time(0)
    f = open("path.txt")
    for line in f:
        print(line.rstrip())
        pose_msg = line.split(",")
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.pose.position.x = float(pose_msg[0])
        pose.pose.position.y = float(pose_msg[1])
        pose.pose.position.z = float(pose_msg[2])
        pose.pose.orientation.x = float(pose_msg[3])
        pose.pose.orientation.y = float(pose_msg[4])
        pose.pose.orientation.z = float(pose_msg[5])
        pose.pose.orientation.w = float(pose_msg[6])
        route.poses.append(pose)
    while not rospy.is_shutdown():
        path_pub.publish(route)
    print("out")    
        
    


if __name__ == '__main__':
    rospy.init_node('path_publisher_node', anonymous=True) 
    
    """if the user didn't enter and argument the node will assume that the 'path.txt' file exists 
    and will publish path message according to the points that are written in the file"""
    if len(sys.argv) != 2:
        publishPath()
        rospy.spin()
    else:
        """if the user entered the number 1 as an argument to the node, the node will read from the file ('path.txt') the points
        and will publish a path message according to those points. If the argument to the node was 0, the node will create path and write the points in the 'path.txt' file """
        if int(sys.argv[1]) == 1:
            print("Publish Path")
            publishPath()
            rospy.spin()
        elif int(sys.argv[1]) == 0:
            print("Create Path")
            pathPublisher = path_publisher()
            rospy.spin()
            pathPublisher.f.close()
            print("Closed File")

