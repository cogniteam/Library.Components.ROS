# Joystick-Teleop-Twist

<img src="./joystick-teleop-twist/nimbusc.jpg" alt="joystick-teleop-twist" width="400"/>

* Dockerhub image https://hub.docker.com/r/cognimbus/joystick-teleop
* Supported architectures <b>arm64/amd64</b>
* ROS version <b>noetic
</b>

# Short description
* nimbus component for controlling your robot with ps3/xbox joystick.
For safety, before you move your robot with the left stick, hold the X(xbox)/Square(ps3) button

# Example usage
```
docker run -it --network=host --privileged cognimbus/joystick-teleop roslaunch teleop_twist_joy teleop.launch joy_config:=xbox joy_dev:=/dev/input/js0
```

# Subscribers
This node has no subscribers


# Publishers
ROS topic | type
--- | ---
/cmd_vel | geometry_msgs/Twist


# Required tf
This node does not require tf


# Provided tf
This node does not provide tf


