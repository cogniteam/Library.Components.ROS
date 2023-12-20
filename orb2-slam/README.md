# Orb2-Slam-D435

<img src="./orb2-slam-d435/nimbusc.gif" alt="orb2-slam-d435" width="400"/>

* Dockerhub image https://hub.docker.com/r/cognimbus/orb2-slam
* Supported architectures <b>arm64/amd64</b>
* ROS version <b>melodic-robot
</b>

# Short description
* ORB-SLAM2 real-time SLAM library for Monocular, Stereo and RGB-D cameras
License: GPLv3
Source: git: https://github.com/cognimbus/nimbus.library/tree/master/Library/Components/orb2-slam

# Example usage
```
docker run -it --network=host cognimbus/orb2-slam roslaunch orb_slam2_ros orb_slam2_d435_rgbd.launch
```

# Subscribers
ROS topic | type
--- | ---
/camera/depth/image_rect_raw | sensor_msgs/Image
/camera/color/image_rect_color | sensor_msgs/Image
/camera/rgb/camera_info | sensor_msgs/CameraInfo


# Publishers
ROS topic | type
--- | ---
/orb_slam2_rgbd/pose | geometry_msgs/PoseStamped
/orb_slam2_rgbd/map_points | sensor_msgs/PointCloud2


# Required tf
This node does not require tf


# Provided tf
This node does not provide tf


