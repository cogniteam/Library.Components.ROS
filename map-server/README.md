# Map-Server

<img src="./map-server/Cogniteam_CMYK_Social_white_on_aubergine copy.jpg" alt="map-server" width="400"/>

* Dockerhub image https://hub.docker.com/r/cognimbus/map-server
* Supported architectures <b>arm64/amd64</b>
* ROS version <b>melodic
</b>

# Short description
* map_server provides the map_server ROS Node, which offers map data as a ROS Service. It also provides the map_saver command-line utility, which allows dynamically generated maps to be saved to file.
License: BSD
GIT:https://github.com/ros-planning/navigation.git

# Example usage
```
docker run -it --network=host -v /opt/nimbus/data/map-server/maps/:/opt/nimbus/data/map-server/maps/ cognimbus/map-server roslaunch map_server map_server.launch publish_rate:=1 --screen
```

# Subscribers
This node has no subscribers


# Publishers
ROS topic | type
--- | ---
/map | nav_msgs/OccupancyGrid
/map_realtime | nav_msgs/OccupancyGrid


# Required tf
This node does not require tf


# Provided tf
This node does not provide tf


