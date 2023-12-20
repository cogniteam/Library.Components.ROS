# Autopilot Bridge

<img src="./autopilot bridge/Autopilot_bridge.png" alt="autopilot bridge" width="400"/>

* Dockerhub image https://hub.docker.com/r/cognimbus/ros1-autopilot-bridge
* Supported architectures <b>arm64/amd64</b>
* ROS version <b>navSatFix
</b>

# Short description
* A ROS bridge to various autopilot protocols. Currently, MAVLink is supported

# Example usage
```
docker run -it --network=host --privileged cognimbus/ros1-autopilot-bridge:navSatFix roslaunch autopilot_bridge autopilot_bridge.launch protocol:=udp ip:=192.168.0.20 port:=10024
```

# Subscribers
ROS topic | type
--- | ---
/autopilot/waypoint_goto | std_msgs/UInt16
/autopilot/mode | std_msgs/String
/autopilot/arm | std_msgs/Bool
/autopilot/guided_goto_NavSatFix | sensor_msgs/NavSatFix
/autopilot/guided_goto | autopilot_bridge/LLA


# Publishers
ROS topic | type
--- | ---
/autopilot/imu | sensor_msgs/Imu
/autopilot/gps | sensor_msgs/NavSatFix
/autopilot/gps_odom | nav_msgs/Odometry
/autopilot/flight_mode | std_msgs/String
/autopilot/voltage_battery | std_msgs/String
/autopilot/current_battery | std_msgs/String
/autopilot/status | autopilot_bridge/Status


# Required tf
This node does not require tf


# Provided tf
This node does not provide tf


