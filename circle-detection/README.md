# Circle-Detection

<img src="./circle-detection/Cogniteam_CMYK_Social_white_on_aubergine.jpg" alt="circle-detection" width="400"/>

* Dockerhub image https://hub.docker.com/r/cognimbus/circle-detection
* Supported architectures <b>amd64/arm64</b>
* ROS version <b>melodic
</b>

# Short description
* Detect the largest circle in video stream
License: BSD
git: https://github.com/cognimbus/nimbus.library/tree/master/Library/Components/depth-2-grayscale

# Example usage
```
docker run -it --network=host cognimbus/circle-detection roslaunch circle_detection circle_detection.launch min_dist_between_two_circles:=1 canny_high_threshold:=100 min_number_of_votes:=150 min_radius:=0 max_radius:=100
```

# Subscribers
ROS topic | type
--- | ---
/usb_cam/image_raw | sensor_msgs/Image


# Publishers
ROS topic | type
--- | ---
/circle_img/compressed | sensor_msgs/CompressedImage
/largest_circle | geometry_msgs/Vector3


# Required tf
This node does not require tf


# Provided tf
This node does not provide tf


