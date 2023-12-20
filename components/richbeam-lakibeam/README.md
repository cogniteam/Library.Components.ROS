# Richbeam-Lakibeam

<img src="./richbeam-lakibeam/lakibeam.png" alt="richbeam-lakibeam" width="400"/>

* Dockerhub image https://hub.docker.com/r/cognimbus/ros1-richbeam-lakibeam-driver
* Supported architectures <b>arm64/amd64/unknown/unknown</b>
* ROS version <b>noetic
</b>

# Short description
* richbeam-lakibeam-lidar driver
License: BSD

# Example usage
```
docker run -it --network=host cognimbus/ros1-richbeam-lakibeam-driver:noetic roslaunch lakibeam1 lakibeam1_scan.launch inverted:=false hostip:=0.0.0.0 port:=2368 angle_offset:=0
```

# Subscribers
This node has no subscribers


# Publishers
ROS topic | type
--- | ---
/scan | sensor_msgs/LaserScan


# Required tf
This node does not require tf


# Provided tf
This node does not provide tf


