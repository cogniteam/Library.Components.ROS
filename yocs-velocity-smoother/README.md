# Yocs-Velocity-Smoother

<img src="./yocs-velocity-smoother/1397206.png" alt="yocs-velocity-smoother" width="400"/>

* Dockerhub image https://hub.docker.com/r/cognimbus/yocs_velocity_smoother
* Supported architectures <b>arm64/amd64</b>
* ROS version <b>melodic
</b>

# Short description
* Bound incoming velocity messages according to robot velocity and acceleration limits.
Author: Jorge Santos Simon
License: BSD
Source: git https://github.com/yujinrobot/yujin_ocs.git

# Example usage
```
docker run -it --network=host cognimbus/yocs_velocity_smoother roslaunch yocs_velocity_smoother velocity_smoother_with_params.launch accel_lim_v:=0.3 accel_lim_w:=3.5 decel_factor:=1 frequency:=20 speed_lim_v:=0.8 speed_lim_w:=5.4 robot_feedback:=0
```

# Subscribers
ROS topic | type
--- | ---
/raw_cmd_vel | geometry_msgs/Twist
/odom | nav_msgs/Odometry


# Publishers
ROS topic | type
--- | ---
/smooth_cmd_vel | geometry_msgs/Twist


# Required tf
This node does not require tf


# Provided tf
This node does not provide tf


