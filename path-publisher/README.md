# Path-Publisher

<img src="./path-publisher/cogniteam.jpg" alt="path-publisher" width="400"/>

* Dockerhub image https://hub.docker.com/r/cognimbus/path-publisher
* Supported architectures <b>arm64/amd64</b>
* ROS version <b>noetic
</b>

# Short description
* path-publisher
 License: BSD

# Example usage
```
docker run -it --network=host --privileged -v /opt/nimbus/data/waypoints/:/path_publisher_ws/src/path_publisher/waypoints/ cognimbus/path-publisher:latest roslaunch path_publisher path_publisher.launch waypoints_file_name:=square.yaml --screen
```

# Subscribers
ROS topic | type
--- | ---
/send_path | std_msgs/Bool


# Publishers
ROS topic | type
--- | ---
/polygon_path | nav_msgs/Path


# Required tf
This node does not require tf


# Provided tf
This node does not provide tf


