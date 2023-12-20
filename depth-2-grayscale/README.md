# Depth-2-Grayscale

<img src="./depth-2-grayscale/Cogniteam_CMYK_Social_white_on_aubergine.jpg" alt="depth-2-grayscale" width="400"/>

* Dockerhub image https://hub.docker.com/r/cognimbus/depth_to_compressed_grayscale_img
* Supported architectures <b>amd64/arm64</b>
* ROS version <b>melodic
</b>

# Short description
* convert depth img (with float data) to_compressed_grayscale_img
git: https://github.com/cognimbus/nimbus.library/tree/master/Library/Components/depth-2-grayscale
License: BSD

# Example usage
```
docker run -it --network=host cognimbus/depth_to_compressed_grayscale_img roslaunch depth_to_grayscale depth_to_grayscale.launch min_gray_scale_value:=0 max_gray_scale_value:=255 max_distance:=3000
```

# Subscribers
ROS topic | type
--- | ---
/zed/zed_node/depth/depth_registered/ | sensor_msgs/Image


# Publishers
ROS topic | type
--- | ---
/grayscale_depth_img/compressed | sensor_msgs/CompressedImage


# Required tf
This node does not require tf


# Provided tf
This node does not provide tf


