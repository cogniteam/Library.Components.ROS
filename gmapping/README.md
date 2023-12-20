# Gmapping

<img src="./gmapping/nimbusc.gif" alt="gmapping" width="400"/>

* Dockerhub image https://hub.docker.com/r/cognimbus/gmapping
* Supported architectures <b>amd64/arm64</b>
* ROS version <b>melodic-ros-core
</b>

# Short description
* OpenSLAM GMapping, Rao-Blackwellized particle filer algorithm
 git: https://github.com/cognimbus/nimbus.library/tree/master/Library/Components/gmapping
License: Apache 2.0

# Example usage
```
docker run -it --network=host cognimbus/gmapping roslaunch gmapping slam_gmapping.launch sigma:=0.1 xmin:=-5 ymin:=-5 xmax:=5 ymax:=5 use_sim_time:=false
```

# Subscribers
ROS topic | type
--- | ---
base_scan | sensor_msgs/LaserScan


# Publishers
ROS topic | type
--- | ---
/map | nav_msgs/OccupancyGrid


# Required tf
odom--->base_link


# Provided tf
map--->base_link
map--->odom


