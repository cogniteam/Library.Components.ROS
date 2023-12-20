# Depth-2-Pcloud

<img src="./depth-2-pcloud/Cogniteam_CMYK_Social_white_on_aubergine.jpg" alt="depth-2-pcloud" width="400"/>

* Dockerhub image https://hub.docker.com/r/cognimbus/depth_to_pcloud_scan
* Supported architectures <b>arm64/amd64</b>
* ROS version <b>melodic
</b>

# Short description
* Convert depth img (with float data) to compressed grayscale image.
git: https://github.com/cognimbus/nimbus.library/tree/master/Library/Components/depth-2-pcloud
License: BSD

# Example usage
```
docker run -it --network=host cognimbus/depth_to_pcloud_scan roslaunch depth_to_pcloud_scan depth_to_pcloud_scan.launch
```

# Subscribers
ROS topic | type
--- | ---
/camera/depth/image_rect_raw | sensor_msgs/Image
/camera/depth/camera_info | sensor_msgs/CameraInfo


# Publishers
ROS topic | type
--- | ---
/pointcloud | sensor_msgs/PointCloud2


# Required tf
This node does not require tf


# Provided tf
This node does not provide tf


