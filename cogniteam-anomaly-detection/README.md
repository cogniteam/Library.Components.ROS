# Cogniteam-Anomaly-Detection

<img src="./cogniteam-anomaly-detection/falling-off-chart.jpg" alt="cogniteam-anomaly-detection" width="400"/>

* Dockerhub image https://hub.docker.com/r/cognimbus/cogniteam-anomaly-detection
* Supported architectures <b>arm64/amd64/unknown/unknown</b>
* ROS version <b>ros-core
</b>

# Short description
* Algorithms of anomaly detection
License: BSD

# Example usage
```
docker run -it --network=host cognimbus/cogniteam-anomaly-detection roslaunch anomaly_detection_launch train.launch
```

# Subscribers
This node has no subscribers


# Publishers
ROS topic | type
--- | ---
/anomalies | std_msgs/String


# Required tf
This node does not require tf


# Provided tf
This node does not provide tf


