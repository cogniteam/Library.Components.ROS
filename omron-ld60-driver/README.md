# Qcr-Omron-Ld60-Driver

<img src="./qcr-omron-ld60-driver/nimbusc.jpg" alt="qcr-omron-ld60-driver" width="400"/>

* Dockerhub image https://hub.docker.com/r/cognimbus/omron_ld60_driver
* Supported architectures <b>amd64</b>
* ROS version <b>melodic
</b>

# Short description
* QCR Omron-LD60 robot driver
git: https://github.com/qcr/ros_omron_agv
License: BSD

# Example usage
```
docker run -it --network=host cognimbus/omron_ld60_driver roslaunch ros_omron_agv omron_bringup.launch host:=10.10.10.10 port:=7272 user:=admin
```

# Subscribers
ROS topic | type
--- | ---
/mobile_base/commands/velocity | geometry_msgs/Twist


# Publishers
ROS topic | type
--- | ---
/amcl_pose | geometry_msgs/PoseWithCovarianceStamped
/laser | sensor_msgs/LaserScan
/laser_low | sensor_msgs/LaserScan


# Required tf
This node does not require tf


# Provided tf
This node does not provide tf


