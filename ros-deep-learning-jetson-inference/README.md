# Ros-Deep-Learning-Jetson-Inference-Detection

<img src="./ros-deep-learning-jetson-inference-detection/deep-vision-primitives.jpg" alt="ros-deep-learning-jetson-inference-detection" width="400"/>

* Dockerhub image https://hub.docker.com/r/cognimbus/ros_deep_learning_jetson_inference
* Supported architectures <b>arm64</b>
* ROS version <b>
</b>

# Short description
* ros deep learning with Nvidia acceleration support
License: MIT

# Example usage
```
docker run -it --network=host --privileged cognimbus/ros_deep_learning_jetson_inference 
```

# Subscribers
ROS topic | type
--- | ---
/camera/color/image_raw | sensor_msgs/Image


# Publishers
ROS topic | type
--- | ---
/detectnet/overlay | sensor_msgs/Image
/detectnet/detections | vision_msgs/Detection2DArray


# Required tf
This node does not require tf


# Provided tf
This node does not provide tf


