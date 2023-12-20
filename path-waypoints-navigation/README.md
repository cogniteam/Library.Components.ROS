# Path-Waypoints-Navigation

<img src="./path-waypoints-navigation/cogniteam.jpg" alt="path-waypoints-navigation" width="400"/>

* Dockerhub image https://hub.docker.com/r/cognimbus/path-waypoints-navigation
* Supported architectures <b>arm64/amd64</b>
* ROS version <b>noetic
</b>

# Short description
* waypoints_navigation
License: BSD

# Example usage
```
docker run -it --network=host cognimbus/path-waypoints-navigation roslaunch waypoints_navigation waypoints_navigation.launch minimal_distance:=-1 wait_for_pause_cmd_enable:=false go_to_next_point_before_reach_current:=false max_distance_before_skip_current_goal:=0.3 --screen
```

# Subscribers
ROS topic | type
--- | ---
/waypoints_route | nav_msgs/Path
/start_mission | std_msgs/Bool
/command_string | std_msgs/String
/move_base/feedback | move_base_msgs/MoveBaseActionFeedback
/move_base/result | move_base_msgs/MoveBaseActionResult
/move_base/status | actionlib_msgs/GoalStatusArray


# Publishers
ROS topic | type
--- | ---
/log | std_msgs/String
/current_goal_reached | std_msgs/Bool
/navigation_done | std_msgs/Bool
/move_base/cancel | actionlib_msgs/GoalID
/move_base/goal | move_base_msgs/MoveBaseActionGoal
/waypoints_markers | visualization_msgs/MarkerArray


# Required tf
odom--->base_link
map--->odom


# Provided tf
This node does not provide tf


