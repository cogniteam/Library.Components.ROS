# Rover-Driver

<img src="./rover-driver/rover-driver.jpg" alt="rover-driver" width="400"/>

* Dockerhub image https://hub.docker.com/r/cognimbus/rover-driver
* Supported architectures <b>arm64/amd64</b>
* ROS version <b>melodic
</b>

# Short description
* Provides an interface between ros and Rover Robotics rover hardware. Inputs to rr_openrover_driver include emergency stop and velocity commands. It outputs diagnostic data such as encoder readings and battery charge. 
License: GPL
GIT:https://github.com/RoverRobotics-archive/rr_openrover_stack.git

# Example usage
```
docker run -it --network=host cognimbus/rover-driver:latest roslaunch roverrobotics_driver zero2.launch
```

# Subscribers
ROS topic | type
--- | ---
/cmd_vel | geometry_msgs/Twist


# Publishers
ROS topic | type
--- | ---
/odom | nav_msgs/Odometry


# Required tf
This node does not require tf


# Provided tf
This node does not provide tf


