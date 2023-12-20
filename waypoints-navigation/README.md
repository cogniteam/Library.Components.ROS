# Waypoints-Navigation

<img src="./waypoints-navigation/Cogniteam_CMYK_Social_white_on_aubergine.jpg" alt="waypoints-navigation" width="400"/>

* Dockerhub image https://hub.docker.com/r/cognimbus/waypoints-navigation
* Supported architectures <b>amd64/arm64</b>
* ROS version <b>melodic
</b>

# Short description
* waypoints_navigation
License: BSD

# Example usage
```
docker run -it --network=host -v /opt/nimbus/data/waypoints/:/turtlebot_laser_navigation_ws/src/turtlebot_laser_navigation/waypoints/ cognimbus/waypoints-navigation roslaunch turtlebot_laser_navigation wait_for_ack_next_point:=false wait:=1 enable_rotate_in_place:=false duration_rotation_seconds:=5 rotation_speed:=0.5 nimbus.launch --screen
```

# Subscribers
ROS topic | type
--- | ---
/move_to_next_point | std_msgs/Bool
/move_base/feedback | move_base_msgs/MoveBaseActionFeedback
/move_base/result | move_base_msgs/MoveBaseActionResult
/move_base/status | actionlib_msgs/GoalStatusArray


# Publishers
ROS topic | type
--- | ---
/mobile_base/commands/velocity | geometry_msgs/Twist
/goals | visualization_msgs/MarkerArray
/start_stop_docking | std_msgs/Bool
/undock_from_charger | std_msgs/Bool
/move_base/cancel | actionlib_msgs/GoalID
/move_base/goal | move_base_msgs/MoveBaseActionGoal


# Required tf
odom--->base_link
map--->odom


# Provided tf
This node does not provide tf


