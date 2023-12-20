# Cogniteam-Person-Follower-Depth

<img src="./cogniteam-person-follower-depth/Cogniteam_CMYK_Social_white_on_aubergine.jpg" alt="cogniteam-person-follower-depth" width="400"/>

* Dockerhub image https://hub.docker.com/r/cognimbus/person-follower
* Supported architectures <b>arm64/amd64</b>
* ROS version <b>melodic
</b>

# Short description
* A person following algorithm that can work with either openvino or ros-deep-learing detections
License: BSD
git: https://github.com/cognimbus/nimbus.library/tree/master/Library/Components/cogniteam-person-follower-depth

# Example usage
```
docker run -it --network=host cognimbus/person-follower roslaunch hupster_detection intel.launch enabled:=true min_speed:=0.1 max_speed:=0.4 max_rotation:=1 min_distance:=1.5 max_distance:=3.5
```

# Subscribers
ROS topic | type
--- | ---
/camera_front/aligned_depth_to_color/image_raw | sensor_msgs/Image
/camera_front/color/camera_info | sensor_msgs/CameraInfo
/vision_msg_detected_objects | vision_msgs/Detection2DArray
/openvino_toolkit/detected_objects | object_msgs/ObjectsInBoxes


# Publishers
ROS topic | type
--- | ---
/navigation_velocity_smoother/raw_cmd_vel | geometry_msgs/Twist
/detected_objects | visualization_msgs/MarkerArray


# Required tf
base_link--->camera_depth_optical_frame


# Provided tf
This node does not provide tf


