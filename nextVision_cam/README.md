# Nextvision_Cam

<img src="./nextVision_cam/nimbusc.jpg" alt="nextVision_cam" width="400"/>

* Dockerhub image https://hub.docker.com/r/cognimbus/ros1-nextvision-cam
* Supported architectures <b>arm64/amd64</b>
* ROS version <b>noetic
</b>

# Short description
* ros node for controll the camera gimbal of nexvision.
License: BSD

# Example usage
```
docker run -it --network=host --privileged cognimbus/ros1-nextvision-cam roslaunch next_vision_cam next_vision.launch ip:=192.168.0.20 port:=10024
```

# Subscribers
ROS topic | type
--- | ---
/next_vision_cam/cmd_vel | Twist


# Publishers
This node has no publishers


# Required tf
This node does not require tf


# Provided tf
This node does not provide tf


