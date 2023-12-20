# Custom-Message-Listener

<img src="./custom-message-listener/nimbusc.jpg" alt="custom-message-listener" width="400"/>

* Dockerhub image https://hub.docker.com/r/cognimbus/custome-message-tutorial
* Supported architectures <b>arm64/amd64</b>
* ROS version <b>noetic
</b>

# Short description
* 

# Example usage
```
docker run -it --network=host cognimbus/custome-message-tutorial roslaunch listener listener.launch
```

# Subscribers
ROS topic | type
--- | ---
/custom_chatter | 


# Publishers
This node has no publishers


# Required tf
This node does not require tf


# Provided tf
This node does not provide tf


# Custom-Message-Talker

<img src="./custom-message-talker/nimbusc.jpg" alt="custom-message-talker" width="400"/>

* Dockerhub image https://hub.docker.com/r/cognimbus/custome-message-tutorial
* Supported architectures <b>arm64/amd64</b>
* ROS version <b>noetic
</b>

# Short description
* The generic talker from the ros1 tutorials using custom messages
License: BSD

# Example usage
```
docker run -it --network=host cognimbus/custome-message-tutorial roslaunch talker talker.launch
```

# Subscribers
This node has no subscribers


# Publishers
ROS topic | type
--- | ---
/custom_chatter | 


# Required tf
This node does not require tf


# Provided tf
This node does not provide tf


