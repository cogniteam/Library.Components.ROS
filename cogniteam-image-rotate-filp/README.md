# Cogniteam-Image-Rotate-Filp

<img src="./cogniteam-image-rotate-filp/Cogniteam_CMYK_Social_white_on_aubergine copy.jpg" alt="cogniteam-image-rotate-filp" width="400"/>

* Dockerhub image https://hub.docker.com/r/cognimbus/image-rotate-filp
* Supported architectures <b>arm64/amd64</b>
* ROS version <b>melodic
</b>

# Short description
* A Component for image rotation or fliping.
git: https://github.com/cognimbus/nimbus.library/tree/master/Library/Components/cogniteam-person-follower-rgb
License: BSD

# Example usage
```
docker run -it --network=host cognimbus/image-rotate-filp roslaunch image_rotate_filp image_rotate_filp.launch angle_rotation_degrees:=180 flip_both:=false flip_vertical:=false flip_horizontal:=false --screen
```

# Subscribers
ROS topic | type
--- | ---
/camera/color/image_raw | sensor_msgs/Image


# Publishers
ROS topic | type
--- | ---
/out_img/ | sensor_msgs/Image
/out_img/compressed/ | sensor_msgs/CompressedImage


# Required tf
This node does not require tf


# Provided tf
This node does not provide tf


