# Openvino-Cpu-Segmentation

<img src="./openvino-cpu-segmentation/opnvino.jpeg" alt="openvino-cpu-segmentation" width="400"/>

* Dockerhub image https://hub.docker.com/r/intelpengo/openvino
* Supported architectures <b>amd64</b>
* ROS version <b>kinetic
</b>

# Short description
* A ROS package to wrap openvino inference engine and get it working with Intel CPU/GPUs.
License: BSD

# Example usage
```
docker run -it --network=host --privileged intelpengo/openvino bash -ic roslaunch vino_launch pengo_segmentation.launch myriad:=false camera_name:=camera
```

# Subscribers
ROS topic | type
--- | ---
/camera/color/image_raw | sensor_msgs/Image


# Publishers
ROS topic | type
--- | ---
/openvino_toolkit/images | sensor_msgs/Image
/openvino_toolkit/detected_objects | object_msgs/ObjectsInBoxes


# Required tf
This node does not require tf


# Provided tf
This node does not provide tf


# Openvino-Cpu-Detection

<img src="./openvino-cpu-detection/opnvino.jpeg" alt="openvino-cpu-detection" width="400"/>

* Dockerhub image https://hub.docker.com/r/intelpengo/openvino
* Supported architectures <b>amd64</b>
* ROS version <b>kinetic
</b>

# Short description
* openvino cpu enabled image recognition
License: Apache 2.0

# Example usage
```
docker run -it --network=host --privileged intelpengo/openvino bash -ic roslaunch vino_launch pengo_detection.launch myriad:=false camera_name:=camera
```

# Subscribers
ROS topic | type
--- | ---
/camera/color/image_raw | sensor_msgs/Image


# Publishers
ROS topic | type
--- | ---
/openvino_toolkit/images | sensor_msgs/Image
/openvino_toolkit/detected_objects | object_msgs/ObjectsInBoxes


# Required tf
This node does not require tf


# Provided tf
This node does not provide tf


# Openvino-Myriad

<img src="./openvino-myriad/opnvino.jpeg" alt="openvino-myriad" width="400"/>

* Dockerhub image https://hub.docker.com/r/intelpengo/openvino
* Supported architectures <b>amd64</b>
* ROS version <b>kinetic
</b>

# Short description
* A ROS package to wrap openvino inference engine and get it working with Myriad. This component allows an easy setup for object detection.
License: BSD

# Example usage
```
docker run -it --network=host --privileged intelpengo/openvino bash -ic roslaunch vino_launch pengo_detection.launch myriad:=true camera_name:=camera
```

# Subscribers
ROS topic | type
--- | ---
/camera/color/image_raw | sensor_msgs/Image


# Publishers
ROS topic | type
--- | ---
/openvino_toolkit/images | sensor_msgs/Image
/openvino_toolkit/detected_objects | object_msgs/ObjectsInBoxes


# Required tf
This node does not require tf


# Provided tf
This node does not provide tf


