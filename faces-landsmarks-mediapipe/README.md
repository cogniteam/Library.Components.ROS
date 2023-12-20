# Faces-Landsmarks-Mediapipe-Amd

<img src="./faces-landsmarks-mediapipe-amd/faces.jpg" alt="faces-landsmarks-mediapipe-amd" width="400"/>

* Dockerhub image https://hub.docker.com/r/cognimbus/faces-landsmarks-mediapipe-amd
* Supported architectures <b>amd64</b>
* ROS version <b>noetic
</b>

# Short description
* google's faces-landsmarks-mediapipe-amd
git: https://github.com/cognimbus/nimbus.library/tree/master/Library/Components/
License: BSD

# Example usage
```
docker run -it --network=host --privileged cognimbus/faces-landsmarks-mediapipe-amd roslaunch face_landsmark faces_landsmarks.launch --screen
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


