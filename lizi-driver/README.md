# Lizi-Driver

<img src="./lizi-driver/lizi.jpeg" alt="lizi-driver" width="400"/>

* Dockerhub image https://hub.docker.com/r/cognimbus/lizi-driver
* Supported architectures <b>amd64</b>
* ROS version <b>kinetic
</b>

# Short description
* Robotican Lizi robot driver
git: https://github.com/cognimbus/nimbus.library/tree/master/Library/Components/lizi-driver
License: BSD

# Example usage
```
docker run -it --network=host cognimbus/lizi-driver roslaunch lizi_hw lizi_hw.launch --screen
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


