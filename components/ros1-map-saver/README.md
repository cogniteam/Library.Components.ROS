# Map-Saver

<img src="./map-saver/Cogniteam_CMYK_Social_white_on_aubergine copy.jpg" alt="map-saver" width="400"/>

* Dockerhub image https://hub.docker.com/r/cognimbus/map-saver
* Supported architectures <b>arm64/amd64</b>
* ROS version <b>noetic</b>

# Short description
* map_saver responsible for saving a map using the map_server package

# Example usage
```
docker run -it --network=host -v /opt/nimbus/data:/opt/nimbus/data cognimbus/map-saver:noetic roslaunch map_saver_node map_saver.launch path:=/opt/nimbus/data map_name:=
```

# Subscribers
ROS topic | type
--- | ---
/map | nav_msgs/OccupancyGrid
/map_saver/save | std_msgs/Bool


# Publishers
This node has no publishers


# Required tf
This node does not require tf


# Provided tf
This node does not provide tf


