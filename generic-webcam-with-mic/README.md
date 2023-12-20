# Generic-Webcam-With-Mic

<img src="./generic-webcam-with-mic/generic-webcam-driver.jpg" alt="generic-webcam-with-mic" width="400"/>

* Dockerhub image https://hub.docker.com/r/cognimbus/generic-webcam-with-mic
* Supported architectures <b>arm64/amd64</b>
* ROS version <b>noetic
</b>

# Short description
* Generic webcam driver
License: BSD

# Example usage
```
docker run -it --network=host --privileged -v /dev/snd:/dev/snd cognimbus/generic-webcam-with-mic:latest roslaunch usb_cam_clear.launch width:=640 quality:=40 height:=480 fps:=15 pixel_format:=yuyv io_method:=userptr frame_id:=camera
```

# Subscribers
This node has no subscribers


# Publishers
ROS topic | type
--- | ---
/usb_cam/image_raw/compressed | sensor_msgs/CompressedImage
/usb_cam/image_raw | sensor_msgs/Image
/audio/audio | audio_common_msgs/AudioData


# Required tf
This node does not require tf


# Provided tf
This node does not provide tf


