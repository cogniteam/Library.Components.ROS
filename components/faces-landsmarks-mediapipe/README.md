# Faces-Landsmarks-Mediapipe

<img src="./faces-landsmarks-mediapipe/faces.jpg" alt="faces-landsmarks-mediapipe" width="400"/>

* Dockerhub image https://hub.docker.com/r/cognimbus/faces-landsmarks-mediapipe
* Supported architectures <b>arm64/amd64/unknown/unknown</b>
* ROS version <b>noetic</b>

# Short description
* google's faces-landsmarks-mediapipe-amd
git: https://github.com/cognimbus/nimbus.library/tree/master/Library/Components/
License: BSD

# Example usage
```
docker run -it --network=host --privileged cognimbus/faces-landsmarks-mediapipe:noetic roslaunch face_landsmark faces_landsmarks.launch --screen
```

# Subscribers
ROS topic | type
--- | ---
//usb_cam/image_raw | sensor_msgs/Image


# Publishers
ROS topic | type
--- | ---
/faces_landsmarks/compressed | sensor_msgs/CompressedImage
/faces_skeleton | std_msgs/String


# Required tf
This node does not require tf


# Provided tf
This node does not provide tf


