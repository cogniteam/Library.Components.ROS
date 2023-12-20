# Leo-Robot-Driver

<img src="./leo-robot-driver/nimbusc.jpg" alt="leo-robot-driver" width="400"/>

* Dockerhub image https://hub.docker.com/r/cognimbus/leo-robot-driver
* Supported architectures <b>arm64</b>
* ROS version <b>noetic</b>

# Short description
* Leo Rover robot driver

# Example usage
```
docker run -it --network=host --privileged -v /dev:/dev cognimbus/leo-robot-driver:noetic roslaunch leo_bringup leo_bringup.launch
```

# Subscribers
ROS topic | type
--- | ---
/cmd_vel | geometry_msgs/Twist


# Publishers
ROS topic | type
--- | ---
/wheel_odom_with_covariance | nav_msgs/Odometry
/imu/data_raw | sensor_msgs/Imu
/camera/image_raw | sensor_msgs/Image
/camera/image_raw/compressed | sensor_msgs/CompressedImage
/camera/image_raw/compressedDepth | sensor_msgs/CompressedImage
/firmware/battery | std_msgs/Float32
/firmware/battery_averaged | std_msgs/Float32


# Required tf
This node does not require tf


# Provided tf
This node does not provide tf


