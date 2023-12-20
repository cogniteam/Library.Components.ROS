# Cogniteam-Coverage-Exploration

<img src="./cogniteam-coverage-exploration/cogniteam_coverage_exploration.jpg" alt="cogniteam-coverage-exploration" width="400"/>

* Dockerhub image https://hub.docker.com/r/cognimbus/cogniteam-coverage-exploration
* Supported architectures <b>arm64/amd64</b>
* ROS version <b>melodic
</b>

# Short description
* Cogniteam exploration is a simple coverage heuristic algorithm that can be used while mapping
License: BSD
git: https://github.com/cognimbus/nimbus.library/tree/master/Library/Components/cogniteam-exploration

# Example usage
```
docker run -it --network=host cognimbus/cogniteam-coverage-exploration:latest roslaunch exploration exploration.launch map_frame:=map searching_radius:=0.3 rate:=0.5 map_dilation_level_m:=0.2 min_percent_coverage:=80 --screen
```

# Subscribers
ROS topic | type
--- | ---
/map | nav_msgs/OccupancyGrid
/move_base/feedback | move_base_msgs/MoveBaseActionFeedback
/move_base/result | move_base_msgs/MoveBaseActionResult
/move_base/status | actionlib_msgs/GoalStatusArray


# Publishers
ROS topic | type
--- | ---
/move_base/cancel | actionlib_msgs/GoalID
/move_base/goal | move_base_msgs/MoveBaseActionGoal
/log | std_msgs/String
/goal_marker | visualization_msgs/MarkerArray
/exploration_img | sensor_msgs/Image
/map_filter | nav_msgs/OccupancyGrid


# Required tf
map--->base_link


# Provided tf
This node does not provide tf


