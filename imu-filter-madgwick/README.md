# Imu-Filter-Madgwick

<img src="./imu-filter-madgwick/cogniteam_imu_tools.jpg" alt="imu-filter-madgwick" width="400"/>

* Dockerhub image https://hub.docker.com/r/cognimbus/imu_tools
* Supported architectures <b>amd64/arm64</b>
* ROS version <b>noetic
</b>

# Short description
* IMU-related filters and visualizers
License: GPL

# Example usage
```
docker run -it --network=host --privileged cognimbus/imu_tools roslaunch imu_filter_madgwick imu_filter_madgwick.launch
```

# Subscribers
ROS topic | type
--- | ---
/imu/data_raw | sensor_msgs/Imu
/imu/mag | sensor_msgs/MagneticField


# Publishers
ROS topic | type
--- | ---
/ImuFilterNodelet/parameter_descriptions | dynamic_reconfigure/ConfigDescription
/ImuFilterNodelet/parameter_updates | dynamic_reconfigure/Config
/imu/data | sensor_msgs/Imu


# Required tf
This node does not require tf


# Provided tf
This node does not provide tf


