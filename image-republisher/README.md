# Image-Republisher

<img src="./image-republisher/Cogniteam_CMYK_Social_white_on_aubergine.jpg" alt="image-republisher" width="400"/>

* Dockerhub image https://hub.docker.com/r/cognimbus/image-republisher
* Supported architectures <b>arm64/amd64</b>
* ROS version <b>melodic
</b>

# Short description
* Republish a message in nimbus, can be parametrized
git: https://github.com/cognimbus/nimbus.library/tree/master/Library/Components/image-republisher
License: BSD

# Example usage
```
docker run -it --network=host cognimbus/image-republisher roslaunch img_republisher img_republisher.launch msgs_per_sec:=5
```

# Subscribers
ROS topic | type
--- | ---
/camera/color/image_raw | sensor_msgs/Image


# Publishers
ROS topic | type
--- | ---
/camera/color/image/republished_img/throttled | sensor_msgs/Image


# Required tf
This node does not require tf


# Provided tf
This node does not provide tf


