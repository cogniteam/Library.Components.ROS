# Cogniteam-Skeleton-Gestures

<img src="./cogniteam-skeleton-gestures/Cogniteam_CMYK_Social_white_on_aubergine.jpg" alt="cogniteam-skeleton-gestures" width="400"/>

* Dockerhub image https://hub.docker.com/r/cognimbus/cogniteam-skeleton-gestures
* Supported architectures <b>arm64/amd64</b>
* ROS version <b>melodic
</b>

# Short description
* An algorithm to create gestures from skeleton detection (can be used with the Nvidia Gem)
git: https://github.com/cognimbus/nimbus.library/tree/master/Library/Components/cogniteam-skeleton-gestures
License: BSD

# Example usage
```
docker run -it --network=host cognimbus/cogniteam-skeleton-gestures roslaunch person_skeleton_follower person_skeleton_follower.launch --screen
```

# Subscribers
ROS topic | type
--- | ---
/webcam/image_raw | sensor_msgs/Image
/skeletons_text | std_msgs/String


# Publishers
ROS topic | type
--- | ---
/gesture_type | std_msgs/String


# Required tf
This node does not require tf


# Provided tf
This node does not provide tf


