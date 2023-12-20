# Hokuyo-Utm-30Lx-Driver

<img src="./hokuyo-utm-30lx-driver/hokuyo-utm-30lx-driver.jpeg" alt="hokuyo-utm-30lx-driver" width="400"/>

* Dockerhub image https://hub.docker.com/r/cognimbus/hokuyo
* Supported architectures <b>arm64/amd64</b>
* ROS version <b>melodic-ros-core
</b>

# Short description
* Hokuyo hokuyo utm-30lx driver
git: https://github.com/cognimbus/nimbus.library/tree/master/Library/Components/hokuyo
License: BSD

# Example usage
```
docker run -it --network=host cognimbus/hokuyo:latest roslaunch urg_node urg_lidar.launch frame_id:=laser publish_intensity:=false angle_min:=-2.2689 angle_max:=2.2689
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


