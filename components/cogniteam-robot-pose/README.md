# Cogniteam-Robot-Pose

<img src="./cogniteam-robot-pose/cogniteam-robot-pose.json" alt="cogniteam-robot-pose" width="400"/>

* Dockerhub image https://hub.docker.com/r/cognimbus/ros1-gateway
* Supported architectures <b>arm64/amd64</b>
* ROS version <b>noetic</b>

# Short description
* Cogniteam platform robot pose publisher

# Example usage
```
docker run -it --network=host cognimbus/ros1-gateway:noetic roslaunch nimbus_robot_pose robot_pose.launch source_frame:=/map target_frame:=base_link
```

# Subscribers
This node has no subscribers


# Publishers
This node has no publishers


# Required tf
This node does not require tf


# Provided tf
This node does not provide tf


