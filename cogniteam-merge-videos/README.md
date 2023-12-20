# Cogniteam-Merge-Videos

<img src="./cogniteam-merge-videos/Cogniteam_CMYK_Social_white_on_aubergine.jpg" alt="cogniteam-merge-videos" width="400"/>

* Dockerhub image https://hub.docker.com/r/cognimbus/cogniteam-merge-videos
* Supported architectures <b>arm64/amd64/unknown/unknown</b>
* ROS version <b>noetic
</b>

# Short description
* ros node that subscribes to a multiple image topic and merge them to one image and publish it.
License: BSD

# Example usage
```
docker run -it --network=host cognimbus/cogniteam-merge-videos:latest roslaunch merge_videos merge_videos.launch
```

# Subscribers
ROS topic | type
--- | ---
/usb_cam_front/image_front | sensor_msgs/Image
/usb_cam_back/image_back | sensor_msgs/Image


# Publishers
ROS topic | type
--- | ---
/merged_image | sensor_msgs/Image
/merged_image/compressed | sensor_msgs/CompressedImage


# Required tf
This node does not require tf


# Provided tf
This node does not provide tf


