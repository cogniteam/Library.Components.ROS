# Ros1-Gateway-Arm32

<img src="./ros1-gateway-arm32/nimbusc.jpg" alt="ros1-gateway-arm32" width="400"/>

* Dockerhub image https://hub.docker.com/r/cognimbus/ros1-gateway-arm32
* Supported architectures <b>arm</b>
* ROS version <b>noetic
</b>

# Short description
* Local ROS master gateway
License: BSD

# Example usage
```
docker run -it --network=host cognimbus/ros1-gateway-arm32 roslaunch empty.launch
```

# Subscribers
This node has no subscribers


# Publishers
ROS topic | type
--- | ---
/scan | LaserScan
/image_raw/compressed | CompressedImage
/usb_cam/image_raw | Image


# Required tf
This node does not require tf


# Provided tf
This node does not provide tf


