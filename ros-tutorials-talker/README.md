# Ros-Tutorials-Talker

<img src="./ros-tutorials-talker/nimbusc.jpg" alt="ros-tutorials-talker" width="400"/>

* Dockerhub image https://hub.docker.com/r/cognimbus/sdk-examples
* Supported architectures <b>arm64/amd64</b>
* ROS version <b>noetic
</b>

# Short description
* The generic talker from the ros1 tutorials
License: BSD

# Example usage
```
docker run -it --network=host cognimbus/sdk-examples roslaunch beginner_tutorials talker.launch
```

# Subscribers
This node has no subscribers


# Publishers
ROS topic | type
--- | ---
/chatter | std_msgs/String


# Required tf
This node does not require tf


# Provided tf
This node does not provide tf


