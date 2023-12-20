# Hector-Mapping-Hd

<img src="./hector-mapping-hd/hector.png" alt="hector-mapping-hd" width="400"/>

* Dockerhub image https://hub.docker.com/r/cognimbus/hector-mapping
* Supported architectures <b>arm64/amd64</b>
* ROS version <b>melodic-ros-core
</b>

# Short description
* 2D laser scan mapping using Hector mapping algorithm (high resolution map)
git: https://github.com/cognimbus/nimbus.library/tree/master/Library/Components/hector-mapping/hector-mapping-hd
License: BSD

# Example usage
```
docker run -it --network=host cognimbus/hector-mapping roslaunch hector_mapping hector_mapping.launch map_resolution:=0.01 map_size:=2048 base_frame:=base_link map_multi_res_levels:=3 scan_subscriber_queue_size:=5 map_frame:=map scan_topic:=/scan output:=screen use_tf_scan_transformation:=true tf_map_scanmatch_transform_frame_name:=laser map_start_x:=0.5 map_start_y:=0.5 max_rotation:=1000 max_translation:=99999
```

# Subscribers
ROS topic | type
--- | ---
scan | sensor_msgs/LaserScan


# Publishers
ROS topic | type
--- | ---
/map | nav_msgs/OccupancyGrid
/pose | geometry_msgs/PoseStamped


# Required tf
This node does not require tf


# Provided tf
This node does not provide tf


# Hector-Mapping

<img src="./hector-mapping/hector.png" alt="hector-mapping" width="400"/>

* Dockerhub image https://hub.docker.com/r/cognimbus/hector-mapping
* Supported architectures <b>arm64/amd64</b>
* ROS version <b>melodic-ros-core
</b>

# Short description
* 2D laser scan mapping using Hector mapping algorithm
git: https://github.com/cognimbus/nimbus.library/tree/master/Library/Components/hector-mapping/hector-mapping-hd
License: BSD

# Example usage
```
docker run -it --network=host cognimbus/hector-mapping roslaunch hector_mapping hector_mapping.launch map_resolution:=0.025 map_size:=2048 base_frame:=base_link map_multi_res_levels:=3 scan_subscriber_queue_size:=5 map_frame:=map output:=screen use_tf_scan_transformation:=true tf_map_scanmatch_transform_frame_name:=laser map_start_x:=0.5 map_start_y:=0.5 max_rotation:=1000 max_translation:=99999
```

# Subscribers
ROS topic | type
--- | ---
scan | sensor_msgs/LaserScan


# Publishers
ROS topic | type
--- | ---
/map | nav_msgs/OccupancyGrid
/slam_out_pose | geometry_msgs/PoseStamped


# Required tf
This node does not require tf


# Provided tf
map--->base_link
map--->odom


