# Cogniteam-Random-Goals

<img src="./cogniteam-random-goals/Cogniteam_CMYK_Social_white_on_aubergine.jpg" alt="cogniteam-random-goals" width="400"/>

* Dockerhub image https://hub.docker.com/r/cognimbus/cogniteam-random-goals
* Supported architectures <b>arm64/amd64/unknown/unknown</b>
* ROS version <b>noetic
</b>

# Short description
* ros program that subscribe to the relevant tf and to the map topic and send the robot to a random goal every time through move-base node.
License: BSD

# Example usage
```
docker run -it --network=host cognimbus/cogniteam-random-goals:latest roslaunch adventech_launch adventech_demo.launch general_max_vel_x:=0.8 general_min_vel_x:=0.2
```

# Subscribers
ROS topic | type
--- | ---
/move_base_simple/goal | geometry_msgs/PoseStamped
/map | nav_msgs/OccupancyGrid
/odom | nav_msgs/Odometry
/scan | sensor_msgs/LaserScan
/start_stop_navigation | std_msgs/Bool


# Publishers
ROS topic | type
--- | ---
/cmd_vel | geometry_msgs/Twist
/nimbus_robot_pose | geometry_msgs/PoseStamped
/nimbus_lidar_pose | geometry_msgs/PoseStamped
/move_base/NavfnROS/plan | nav_msgs/Path
/move_base/local_costmap/costmap | nav_msgs/OccupancyGrid
/move_base/current_goal | geometry_msgs/PoseStamped
/robot_state | std_msgs/String


# Required tf
odom--->base_link
map--->odom


# Provided tf
This node does not provide tf


