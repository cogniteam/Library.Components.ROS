# Dfki-Mir-Driver

<img src="./dfki-mir-driver/nimbusc.jpg" alt="dfki-mir-driver" width="400"/>

* Dockerhub image https://hub.docker.com/r/cognimbus/dfki_mir_driver
* Supported architectures <b>amd64</b>
* ROS version <b>melodic
</b>

# Short description
* DFKI mir robot driver
git: https://github.com/dfki-ric/mir_robot
License: BSD

# Example usage
```
docker run -it --network=host cognimbus/dfki_mir_driver roslaunch mir_driver mir.launch tf_prefix:= mir_hostname:=192.168.12.20 disable_map:=false
```

# Subscribers
ROS topic | type
--- | ---
/mobile_base/commands/velocity | geometry_msgs/Twist


# Publishers
This node has no publishers


# Required tf
This node does not require tf


# Provided tf
This node does not provide tf


