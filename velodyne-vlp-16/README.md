# Velodyne-Vlp-16

<img src="./velodyne-vlp-16/lidar.jpeg" alt="velodyne-vlp-16" width="400"/>

* Dockerhub image https://hub.docker.com/r/cognimbus/velodyne-vlp-16
* Supported architectures <b>amd64/arm64</b>
* ROS version <b>melodic
</b>

# Short description
* ROS support for the Velodyne 3D LIDAR.
Author: Jack O'Quin
License: BSD
Source: git https://github.com/ros-drivers/velodyne.git

# Example usage
```
docker run -it --network=host cognimbus/velodyne-vlp-16 roslaunch velodyne_pointcloud VLP16_points.launch frame_id:=laser device_ip:=192.168.1.201 --screen
```

# Subscribers
This node has no subscribers


# Publishers
ROS topic | type
--- | ---
/scan | sensor_msgs/LaserScan
/velodyne_points | sensor_msgs/PointCloud2


# Required tf
This node does not require tf


# Provided tf
This node does not provide tf


