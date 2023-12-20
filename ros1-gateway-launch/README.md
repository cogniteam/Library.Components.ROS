# Ros1-Gateway-Launch

<img src="./ros1-gateway-launch/nimbusc.jpg" alt="ros1-gateway-launch" width="400"/>

* Dockerhub image https://hub.docker.com/r/cognimbus/ros1-gateway
* Supported architectures <b>arm64/amd64</b>
* ROS version <b>noetic
</b>

# Short description
* Local ROS master gateway
License: BSD

# Example usage
```
docker run -it --network=host cognimbus/ros1-gateway roslaunch empty empty.launch
```

# Subscribers
ROS topic | type
--- | ---
/nimbus_gateway/output/laser_scan | LaserScan
/nimbus_gateway/output/camera/image_raw/compressed | CompressedImage


# Publishers
ROS topic | type
--- | ---
/nimbus_gateway/input/string_command | String
/nimbus_gateway/input/cmd_vel | Twist


# Required tf
odom--->base_link


# Provided tf
This node does not provide tf


