# Spot-Bridge

<img src="./spot-bridge/spot.jpeg" alt="spot-bridge" width="400"/>

* Dockerhub image https://hub.docker.com/r/cognimbus/spot_ws
* Supported architectures <b>arm64/amd64/unknown/unknown</b>
* ROS version <b>noetic
</b>

# Short description
* ros noetic spot-bridge.
License: LGPL
GIT: https://github.com/heuristicus/spot_ros

# Example usage
```
docker run -it --network=host --privileged cognimbus/spot_ws:noetic roslaunch spot_driver driver.launch username:=user password:=12345 hostname:=192.168.80.3 --screen
```

# Subscribers
ROS topic | type
--- | ---
/spot/cmd_vel | geometry_msgs/Twist


# Publishers
This node has no publishers


# Required tf
This node does not require tf


# Provided tf
This node does not provide tf


