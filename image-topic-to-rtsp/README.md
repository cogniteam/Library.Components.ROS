# Image-Topic-To-Rtsp

<img src="./image-topic-to-rtsp/Cogniteam_CMYK_Social_white_on_aubergine.jpg" alt="image-topic-to-rtsp" width="400"/>

* Dockerhub image https://hub.docker.com/r/cognimbus/image-topic-to-rtsp
* Supported architectures <b>arm64/amd64</b>
* ROS version <b>noetic
</b>

# Short description
* ROS1 node that subscribes to image topic and publish it through RTSP server.

# Example usage
```
docker run -it --network=host cognimbus/image-topic-to-rtsp roslaunch ros_rtsp rtsp_streams.launch numOfStreams:=4 mountpoint:=/cam topicPrefix:=/usb_cam/image_raw port:=8554
```

# Subscribers
ROS topic | type
--- | ---
/usb_cam/image_raw1 | sensor_msgs/Image
/usb_cam/image_raw2 | sensor_msgs/Image
/usb_cam/image_raw3 | sensor_msgs/Image
/usb_cam/image_raw4 | sensor_msgs/Image


# Publishers
This node has no publishers


# Required tf
This node does not require tf


# Provided tf
This node does not provide tf


