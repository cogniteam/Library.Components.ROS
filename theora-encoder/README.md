# Theora-Encoder

<img src="./theora-encoder/Theora_logo_2007.svg.png" alt="theora-encoder" width="400"/>

* Dockerhub image https://hub.docker.com/r/cognimbus/theora-encoder
* Supported architectures <b>arm64/amd64</b>
* ROS version <b>melodic
</b>

# Short description
* Theora_image_transport provides a plugin to image_transport for transparently sending an image stream encoded with the Theora codec.theora_image_transport only works with 8-bit color or grayscale images.
Author: Patrick Mihelich, Ethan Dreyfuss
License: BSD
Source: git https://github.com/ros-perception/image_transport_plugins.git

# Example usage
```
docker run -it --network=host cognimbus/theora-encoder:latest roslaunch theora_info theora_info_publisher.launch optimize_for:=1 target_bitrate:=800000 quality:=31 keyframe_frequency:=64 time_interval:=1
```

# Subscribers
ROS topic | type
--- | ---
/usb_cam/image_raw | sensor_msgs/Image


# Publishers
ROS topic | type
--- | ---
/video/theora | theora_image_transport/Packet


# Required tf
This node does not require tf


# Provided tf
This node does not provide tf


