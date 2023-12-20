# Cogniteam-Person-Follower-Rgb

<img src="./cogniteam-person-follower-rgb/Cogniteam_CMYK_Social_white_on_aubergine.jpg" alt="cogniteam-person-follower-rgb" width="400"/>

* Dockerhub image https://hub.docker.com/r/cognimbus/person_follower_rgb_camera
* Supported architectures <b>arm64/amd64</b>
* ROS version <b>melodic
</b>

# Short description
* A person following algorithm that can work with either openvino or ros-deep-learing detections
git: https://github.com/cognimbus/nimbus.library/tree/master/Library/Components/cogniteam-person-follower-rgb
License: BSD

# Example usage
```
docker run -it --network=host cognimbus/person_follower_rgb_camera roslaunch person_follower_rgb person_follower_rgb.launch target:=person focal_length:=740 known_target_width_cm:=50 min_distance:=0.5 max_distance:=1.5 speed:=0.1 box_percentage_from_image_to_stop:=0.8 angular_scale_factor:=1 enable_gestures:=false --screen
```

# Subscribers
ROS topic | type
--- | ---
/openvino_toolkit/images | sensor_msgs/Image
/vision_msg_detected_objects | vision_msgs/Detection2DArray
/openvino_toolkit/detected_objects | object_msgs/ObjectsInBoxes
/gesture_type | std_msgs/String
/skeletons_text | std_msgs/String


# Publishers
ROS topic | type
--- | ---
/mobile_base/commands/velocity | geometry_msgs/Twist
/debug_img/compressed/ | sensor_msgs/CompressedImage


# Required tf
This node does not require tf


# Provided tf
This node does not provide tf


