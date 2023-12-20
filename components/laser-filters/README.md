# Laser-Filters

<img src="./laser-filters/Cogniteam_CMYK_Social_white_on_aubergine.jpg" alt="laser-filters" width="400"/>

* Dockerhub image https://hub.docker.com/r/cognimbus/laser-filters
* Supported architectures <b>arm64/amd64</b>
* ROS version <b>noetic</b>

# Short description
* ros program that filters the laser scan.
License: BSD

# Example usage
```
docker run -it --network=host cognimbus/laser-filters:noetic roslaunch laser_filters laser_scan_filter.launch laser_filter_lower_threshold:=0.3
```

# Subscribers
ROS topic | type
--- | ---
/scan | sensor_msgs/LaserScan


# Publishers
ROS topic | type
--- | ---
/scan_filtered | sensor_msgs/LaserScan


# Required tf
This node does not require tf


# Provided tf
This node does not provide tf


