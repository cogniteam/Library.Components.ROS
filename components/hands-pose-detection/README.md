# Hands-Pose-Detection

<img src="./hands-pose-detection/hand_landmarks.png" alt="hands-pose-detection" width="400"/>

* Dockerhub image https://hub.docker.com/r/cognimbus/hands-pose-detection
* Supported architectures <b>arm64/amd64/unknown/unknown</b>
* ROS version <b>noetic</b>

# Short description
* google's hands-pose-detection
git: https://github.com/cognimbus/nimbus.library/tree/master/Library/Components/
License: BSD

# Example usage
```
docker run -it --network=host --privileged cognimbus/hands-pose-detection:noetic roslaunch hands_pose_detectors hands_pose_detectors.launch --screen
```

# Subscribers
ROS topic | type
--- | ---
//usb_cam/image_raw | sensor_msgs/Image


# Publishers
ROS topic | type
--- | ---
/hands_img/compressed | sensor_msgs/CompressedImage
/hands_skeleton | std_msgs/String


# Required tf
This node does not require tf


# Provided tf
This node does not provide tf


