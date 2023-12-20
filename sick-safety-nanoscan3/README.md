# Nanoscan3

<img src="./nanoscan3/nanoscan3.png" alt="nanoscan3" width="400"/>

* Dockerhub image https://hub.docker.com/r/cognimbus/sick-safety-nanoscan3
* Supported architectures <b>amd64/arm64</b>
* ROS version <b>noetic
</b>

# Short description
* sick-safety-nanoscan3 driver
License: BSD

# Example usage
```
docker run -it --network=host cognimbus/sick-safety-nanoscan3 roslaunch sick_safetyscanners sick_safetyscanners.launch frame_id:=laser sensor_ip:=192.168.1.10 host_ip:=192.168.1.9 --screen
```

# Subscribers
This node has no subscribers


# Publishers
ROS topic | type
--- | ---
/scan | sensor_msgs/LaserScan


# Required tf
This node does not require tf


# Provided tf
This node does not provide tf


